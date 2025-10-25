# detect_picamera2_cone.py
# Picamera2 + PyCoral + OpenCV
# Usa por defecto: ssd_mobilenet_v1_cone_edgetpu.tflite y labels.txt
# Carpeta: /home/hilax/google-coral3/tflite/python/examples/detection/

import argparse
import os
import cv2
import numpy as np
from picamera2 import Picamera2

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter, run_inference

# Tamaño del preview en pantalla
DISPLAY_SIZE = (640, 480)

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_model = os.path.join(here, 'ssd_mobilenet_v1_cone_edgetpu.tflite')
    default_labels = os.path.join(here, 'labels.txt')

    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=default_model,
                        help='Ruta al modelo .tflite (EdgeTPU)')
    parser.add_argument('--labels', default=default_labels,
                        help='Ruta a labels.txt')
    parser.add_argument('--threshold', type=float, default=0.35,
                        help='Confianza mínima para dibujar detecciones')
    parser.add_argument('--top_k', type=int, default=10,
                        help='Máximo de detecciones a mostrar')
    args = parser.parse_args()

    print(f'-> Modelo: {args.model}')
    print(f'-> Labels: {args.labels}')
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = read_label_file(args.labels)

    in_w, in_h = input_size(interpreter)  # p.ej., 300x300
    print(f'-> Tamaño de entrada del modelo: {in_w}x{in_h}')

    # Configurar cámara CSI
    picam2 = Picamera2()
    cfg = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": DISPLAY_SIZE}
    )
    picam2.configure(cfg)
    picam2.start()

    window = 'Cone detection (q para salir)'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, DISPLAY_SIZE[0], DISPLAY_SIZE[1])

    try:
        while True:
            # Frame RGB del sensor
            rgb = picam2.capture_array()

            # Redimensionar SOLO para inferencia
            rgb_small = cv2.resize(rgb, (in_w, in_h), interpolation=cv2.INTER_LINEAR)
            if rgb_small.dtype != np.uint8:
                rgb_small = rgb_small.astype(np.uint8)

            # Inference en Edge TPU
            run_inference(interpreter, rgb_small.tobytes())
            objs = get_objects(interpreter, args.threshold)[:args.top_k]

            # Dibujar resultados en el frame mostrado
            draw_detections(rgb, (in_w, in_h), objs, labels)

            cv2.imshow(window, rgb)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        picam2.stop()
        cv2.destroyAllWindows()


def draw_detections(img_rgb, inference_size, objs, labels):
    """Dibuja bbox y etiquetas en un frame RGB."""
    h, w, _ = img_rgb.shape
    sx, sy = w / inference_size[0], h / inference_size[1]

    for obj in objs:
        bbox = obj.bbox.scale(sx, sy)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        score = int(100 * obj.score)
        cls_name = labels.get(obj.id, str(obj.id))
        label = f'{cls_name} {score}%'

        # Caja más visible
        cv2.rectangle(img_rgb, (x0, y0), (x1, y1), (0, 255, 0), 3)  # verde, grosor 3

        # Texto con contorno para buena legibilidad
        y_text = max(0, y0 - 10)
        cv2.putText(img_rgb, label, (x0, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)      # sombra negra
        cv2.putText(img_rgb, label, (x0, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)  # blanco


if __name__ == '__main__':
    main()
