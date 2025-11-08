#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Flask + OpenCV + PyCoral: transmite RGB (/dev/video4) con detecciones en MJPEG.
# Incluye: --no_det (solo cámara), logs y manejo de errores para que el stream no se caiga.

import os, sys, argparse, threading, time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, jsonify

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter, run_inference

def draw_detections(img_bgr, inference_size, objs, labels):
    h, w, _ = img_bgr.shape
    sx, sy = w / inference_size[0], h / inference_size[1]
    for obj in objs:
        bb = obj.bbox.scale(sx, sy)
        x0, y0 = int(bb.xmin), int(bb.ymin)
        x1, y1 = int(bb.xmax), int(bb.ymax)
        score = int(100 * obj.score)
        name = labels.get(obj.id, str(obj.id))
        cv2.rectangle(img_bgr, (x0, y0), (x1, y1), (0, 255, 0), 3)
        txt = f"{name} {score}%"
        ytxt = max(0, y0 - 8)
        cv2.putText(img_bgr, txt, (x0, ytxt),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(img_bgr, txt, (x0, ytxt),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)

def open_capture(dev, width, height, fps, fourccs=("MJPG","YUYV")):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    for fcc in fourccs:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        if cap.isOpened():
            print(f"[cam] Abierto {dev} {width}x{height}@{fps} {fcc}")
            return cap, fcc
    cap.release()
    return None, None

def build_app(args):
    # Carga modelo / labels salvo que pidan no detectar
    if not args.no_det:
        interpreter = make_interpreter(args.model)
        interpreter.allocate_tensors()
        labels = read_label_file(args.labels)
        in_w, in_h = input_size(interpreter)
        print(f"[tpu] EdgeTPU listo. Input modelo: {in_w}x{in_h}")
    else:
        interpreter = None
        labels = {}
        in_w, in_h = args.width, args.height

    # Cámara
    cap, fcc = open_capture(args.device, args.width, args.height, args.fps)
    if cap is None:
        sys.exit(f"No pude abrir {args.device}. Revisa v4l2-ctl --list-formats-ext.")

    app = Flask(__name__)
    cap_lock = threading.Lock()
    infer_lock = threading.Lock()

    HTML = """
    <!doctype html>
    <title>Detección en vivo</title>
    <style> body{background:#111;color:#eee;font-family:sans-serif}
            .wrap{max-width:980px;margin:24px auto;text-align:center}
            img{width:100%;height:auto;border-radius:12px}
    </style>
    <div class="wrap">
      <h2>Detección (RGB RealSense) — {{wh}} @ {{fps}} FPS</h2>
      <img src="/video_feed">
      <p>Ctrl+C en la terminal para detener.</p>
    </div>"""

    @app.route("/")
    def index():
        return render_template_string(HTML, wh=f"{args.width}x{args.height}", fps=args.fps)

    @app.route("/health")
    def health():
        return jsonify(status="ok")

    @app.route("/video_feed")
    def video_feed():
        print("[http] Cliente conectado a /video_feed")
        def gen():
            objs_cache = []   # mantiene últimas detecciones
            f, t0 = 0, time.time()
            while True:
                with cap_lock:
                    ok, frame = cap.read()
                if not ok:
                    print("[cam] frame vacío, reintentando...")
                    time.sleep(0.01)
                    continue

                f += 1
                # Detección (si procede)
                if not args.no_det:
                    try:
                        if f % args.detect_every == 0:
                            # preparar input
                            rgb_small = cv2.resize(frame, (in_w, in_h), interpolation=cv2.INTER_LINEAR)
                            rgb_small = cv2.cvtColor(rgb_small, cv2.COLOR_BGR2RGB)
                            if rgb_small.dtype != np.uint8:
                                rgb_small = rgb_small.astype(np.uint8)
                            # inferencia
                            with infer_lock:
                                run_inference(interpreter, rgb_small.tobytes())
                                objs_cache = get_objects(interpreter, args.threshold)[:args.top_k]
                    except Exception as e:
                        # No mates el stream si algo falla en la TPU
                        print(f"[tpu] ERROR en inferencia: {e}")
                # Dibuja
                try:
                    if not args.no_det:
                        draw_detections(frame, (in_w, in_h), objs_cache, labels)
                except Exception as e:
                    print(f"[draw] ERROR: {e}")

                # FPS log cada ~2s
                if f % 60 == 0:
                    dt = time.time() - t0
                    if dt > 0:
                        print(f"[fps] {60/dt:.1f} fps")
                    t0 = time.time()

                # Codifica JPG
                ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ok:
                    print("[jpg] fallo al codificar")
                    continue
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.teardown_appcontext
    def cleanup(exc):
        try:
            with cap_lock: cap.release()
        except: pass

    return app

def parse_args():
    here = os.path.dirname(os.path.abspath(__file__))
    default_model  = os.path.join(here, "ssd_mobilenet_v1_cone700_edgetpu.tflite")
    default_labels = os.path.join(here, "labels700.txt")
    p = argparse.ArgumentParser()
    p.add_argument("--device", default="/dev/video4")
    p.add_argument("--model", default=default_model)
    p.add_argument("--labels", default=default_labels)
    p.add_argument("--threshold", type=float, default=0.35)
    p.add_argument("--top_k", type=int, default=10)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--detect_every", type=int, default=3)
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--no_det", action="store_true", help="sirve frames sin detección")
    return p.parse_args()

if __name__ == "__main__":
    args = parse_args()
    app = build_app(args)
    app.run(host=args.host, port=args.port, debug=False, threaded=True)
