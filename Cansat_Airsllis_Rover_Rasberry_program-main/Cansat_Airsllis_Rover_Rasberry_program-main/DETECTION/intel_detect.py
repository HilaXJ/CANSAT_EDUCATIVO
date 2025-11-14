# detect_realsense_rgb.py
# UVC (Intel RealSense RGB en /dev/video4) + PyCoral + OpenCV
# Muestra solo RGB y corre detección con EdgeTPU

import argparse
import os
import sys
import cv2
import numpy as np

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter, run_inference


def open_capture(dev, width, height, fps, fourcc_try=("MJPG", "YUYV")):
    """Intenta abrir /dev/videoX con MJPG y, si no, YUYV."""
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    ok = False
    for fcc in fourcc_try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        ok = cap.isOpened()
        if ok:
            print(f"-> Abierto {dev} {width}x{height}@{fps} con {fcc}")
            return cap, fcc
    cap.release()
    return None, None


def draw_detections(img_bgr, inference_size, objs, labels):
    """Dibuja bbox y etiquetas en el frame BGR mostrado."""
    h, w, _ = img_bgr.shape
    sx, sy = w / inference_size[0], h / inference_size[1]

    for obj in objs:
        bbox = obj.bbox.scale(sx, sy)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)
        score = int(100 * obj.score)
        cls_name = labels.get(obj.id, str(obj.id))
        label = f'{cls_name} {score}%'

        cv2.rectangle(img_bgr, (x0, y0), (x1, y1), (0, 255, 0), 3)
        y_text = max(0, y0 - 10)
        cv2.putText(img_bgr, label, (x0, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img_bgr, label, (x0, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_model  = os.path.join(here, 'ssd_mobilenet_v1_cone700_edgetpu.tflite')
    default_labels = os.path.join(here, 'labels700.txt')

    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='/dev/video4', help='Nodo UVC RGB (p.ej. /dev/video4)')
    parser.add_argument('--model',  default=default_model,  help='Ruta al modelo .tflite (EdgeTPU)')
    parser.add_argument('--labels', default=default_labels, help='Ruta a labels.txt')
    parser.add_argument('--threshold', type=float, default=0.6, help='Confianza mínima')
    parser.add_argument('--top_k',    type=int,   default=10,   help='Máximo de detecciones')
    parser.add_argument('--width',    type=int,   default=640)
    parser.add_argument('--height',   type=int,   default=480)
    parser.add_argument('--fps',      type=int,   default=60)
    args = parser.parse_args()

    print(f'-> Modelo: {args.model}')
    print(f'-> Labels: {args.labels}')

    # Intérprete EdgeTPU
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = read_label_file(args.labels)
    in_w, in_h = input_size(interpreter)  # p.ej. 300x300
    print(f'-> Entrada del modelo: {in_w}x{in_h}')

    # Cámara UVC (RealSense RGB)
    cap, fcc = open_capture(args.device, args.width, args.height, args.fps)
    if cap is None:
        sys.exit(f"No pude abrir {args.device} con MJPG ni YUYV. "
                 "Revisa v4l2-ctl --list-formats-ext.")

    window = 'Cone detection (RGB RealSense) - q para salir'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width, args.height)

    try:
        while True:
            ok, frame_bgr = cap.read()
            if not ok:
                print("Frame vacío; cambia formato/resolución.")
                break

            # Prepara entrada para el modelo: RGB uint8 y tamaño in_w x in_h
            rgb_small = cv2.resize(frame_bgr, (in_w, in_h), interpolation=cv2.INTER_LINEAR)
            rgb_small = cv2.cvtColor(rgb_small, cv2.COLOR_BGR2RGB)
            if rgb_small.dtype != np.uint8:
                rgb_small = rgb_small.astype(np.uint8)

            # Inference en EdgeTPU
            run_inference(interpreter, rgb_small.tobytes())
            objs = get_objects(interpreter, args.threshold)[:args.top_k]

            # Dibuja resultados sobre el frame mostrado (BGR)
            draw_detections(frame_bgr, (in_w, in_h), objs, labels)

            cv2.imshow(window, frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
