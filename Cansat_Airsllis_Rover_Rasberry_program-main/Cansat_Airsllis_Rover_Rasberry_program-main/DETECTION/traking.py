#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Detección esporádica (PyCoral) + tracking ligero (Lucas–Kanade) en /dev/video4 (RGB RealSense)
# NO requiere opencv-contrib. Funciona con opencv-python estándar.

import os, sys, time, argparse
import cv2
import numpy as np

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter, run_inference

# --------- Tracker ligero (sin contrib) ----------
class LKBoxTracker:
    """Tracker de caja con Lucas–Kanade (optical flow) + re-siembra de puntos."""
    def __init__(self):
        self.prev_gray = None
        self.prev_pts  = None
        self.box       = None       # (x,y,w,h)
        self._t        = 0

    def init(self, frame, box_xywh):
        x, y, w, h = map(int, box_xywh)
        self.box = (x, y, max(1, w), max(1, h))
        self.prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        roi = self.prev_gray[y:y+h, x:x+w]
        pts = cv2.goodFeaturesToTrack(
            roi, maxCorners=100, qualityLevel=0.01, minDistance=5, blockSize=7
        )
        if pts is None or len(pts) < 8:
            self.prev_pts = None
            return False
        self.prev_pts = (pts + np.array([[x, y]], dtype=np.float32))
        return True

    def update(self, frame):
        if self.prev_pts is None or self.box is None:
            return False, self.box
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        nxt, st, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        good_prev = self.prev_pts[st.flatten() == 1] if st is not None else None
        good_next = nxt[st.flatten() == 1]          if st is not None and nxt is not None else None
        if good_prev is None or good_next is None or len(good_next) < 8:
            return False, self.box

        # Desplazamiento robusto por mediana
        dxy = np.median(good_next - good_prev, axis=0)
        x, y, w, h = self.box
        x = int(round(x + float(dxy[0])))
        y = int(round(y + float(dxy[1])))
        self.box = (max(0, x), max(0, y), w, h)

        self.prev_gray = gray
        self.prev_pts  = good_next.reshape(-1, 1, 2)

        # Re-siembra periódica o si quedan pocos puntos
        self._t += 1
        if len(self.prev_pts) < 25 or (self._t % 15 == 0):
            rx, ry, rw, rh = self.box
            roi = gray[ry:ry+rh, rx:rx+rw]
            pts = cv2.goodFeaturesToTrack(
                roi, maxCorners=100, qualityLevel=0.01, minDistance=5, blockSize=7
            )
            if pts is not None and len(pts) >= 8:
                self.prev_pts = (pts + np.array([[rx, ry]], dtype=np.float32))
            self._t = 0

        return True, self.box

# --------------- Utilidades ---------------
def iou(a, b):
    """IoU entre dos cajas (x,y,w,h)."""
    ax0, ay0, aw, ah = a; ax1, ay1 = ax0+aw, ay0+ah
    bx0, by0, bw, bh = b; bx1, by1 = bx0+bw, by0+bh
    x0, y0 = max(ax0, bx0), max(ay0, by0)
    x1, y1 = min(ax1, bx1), min(ay1, by1)
    inter = max(0, x1 - x0) * max(0, y1 - y0)
    area_a, area_b = aw*ah, bw*bh
    denom = area_a + area_b - inter + 1e-6
    return inter / denom

def choose_target(objs, labels, infer_size, frame_shape, prev_box=None):
    """Elige detección más consistente con prev_box; si no, la de mayor score.
       Devuelve (score, (x,y,w,h), label_str) o None."""
    h, w = frame_shape[:2]
    sx, sy = w / infer_size[0], h / infer_size[1]
    cand = []
    for o in objs:
        bb = o.bbox.scale(sx, sy)
        x0, y0 = int(bb.xmin), int(bb.ymin)
        x1, y1 = int(bb.xmax), int(bb.ymax)
        box = (max(0, x0), max(0, y0), max(1, x1 - x0), max(1, y1 - y0))
        cand.append((o.score, box, labels.get(o.id, str(o.id))))
    if not cand:
        return None

    if prev_box is not None:
        cand.sort(key=lambda t: iou(prev_box, t[1]), reverse=True)
        if iou(prev_box, cand[0][1]) > 0.1:
            return cand[0]

    cand.sort(key=lambda t: t[0], reverse=True)
    return cand[0]

def draw_box(img, box, label, color=(0, 255, 0)):
    x, y, w, h = map(int, box)
    cv2.rectangle(img, (x, y), (x + w, y + h), color, 3)
    cv2.putText(img, label, (x, max(0, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, label, (x, max(0, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

# --------------- Main ---------------------
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_model  = os.path.join(here, 'ssd_mobilenet_v1_cone700_edgetpu.tflite')
    default_labels = os.path.join(here, 'labels700.txt')

    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='/dev/video4', help='Nodo UVC RGB (p.ej. /dev/video4)')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--model', default=default_model)
    ap.add_argument('--labels', default=default_labels)
    ap.add_argument('--threshold', type=float, default=0.35)
    ap.add_argument('--top_k', type=int, default=10)
    ap.add_argument('--detect_every', type=int, default=10, help='Detectar cada N frames (resto: track)')
    args = ap.parse_args()

    # Coral
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = read_label_file(args.labels)
    in_w, in_h = input_size(interpreter)
    print(f'EdgeTPU listo. Input modelo: {in_w}x{in_h}')

    # Cámara UVC (intenta MJPG y cae a YUYV)
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    opened = False
    for fourcc in ('MJPG', 'YUYV'):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS,          args.fps)
        if cap.isOpened():
            opened = True
            print(f'Abierto {args.device} {args.width}x{args.height}@{args.fps} {fourcc}')
            break
    if not opened:
        sys.exit(f'No pude abrir {args.device}. Revisa v4l2-ctl --list-formats-ext.')

    win = 'Detect+Track (q para salir)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, args.width, args.height)

    tracker = None
    tracked_box = None
    tracked_label = ''
    lost = 0
    f = 0
    t_fps = time.time()
    fps_counter = 0
    fps_show = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            f += 1
            fps_counter += 1
            if fps_counter >= 30:
                dt = time.time() - t_fps
                if dt > 0:
                    fps_show = fps_counter / dt
                t_fps = time.time()
                fps_counter = 0

            need_detect = (tracker is None) or (f % args.detect_every == 0) or (lost >= 3)

            if need_detect:
                rgb = cv2.cvtColor(cv2.resize(frame, (in_w, in_h)), cv2.COLOR_BGR2RGB)
                run_inference(interpreter, rgb.tobytes())
                objs = get_objects(interpreter, args.threshold)[:args.top_k]

                pick = choose_target(objs, labels, (in_w, in_h), frame.shape, tracked_box)
                if pick:
                    score, box, lbl = pick
                    tk = LKBoxTracker()
                    if tk.init(frame, box):
                        tracker = tk
                        tracked_box = box
                        tracked_label = f'{lbl} {int(score*100)}%'
                        lost = 0
                    else:
                        tracker = None
                        tracked_box = None
                        tracked_label = ''
                        lost = 99  # fuerza otra detección pronto
                else:
                    tracker = None
                    tracked_box = None
                    tracked_label = ''
                    lost = 99
            else:
                ok_track, box = tracker.update(frame)
                if ok_track:
                    tracked_box = box
                    lost = 0
                else:
                    lost += 1

            # Dibujo
            overlay = frame
            if tracked_box is not None:
                prefix = 'DETECT ' if need_detect else 'TRACK '
                draw_box(overlay, tracked_box, prefix + tracked_label)
                # Punto central + error para control
                x, y, w, h = map(int, tracked_box)
                cx, cy = x + w // 2, y + h // 2
                cv2.circle(overlay, (cx, cy), 4, (0, 255, 255), -1)
                err_x = cx - overlay.shape[1] // 2
                err_y = cy - overlay.shape[0] // 2
                cv2.putText(overlay, f'err_x={err_x}  err_y={err_y}',
                            (10, overlay.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(overlay, f'err_x={err_x}  err_y={err_y}',
                            (10, overlay.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

            # FPS en overlay
            cv2.putText(overlay, f'{fps_show:.1f} FPS', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(overlay, f'{fps_show:.1f} FPS', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow(win, overlay)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
