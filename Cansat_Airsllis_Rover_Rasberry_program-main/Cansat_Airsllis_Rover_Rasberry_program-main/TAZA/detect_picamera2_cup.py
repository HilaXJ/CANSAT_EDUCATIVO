# detect_picamera2_cup.py
# Picamera2 + PyCoral + OpenCV
# - Un solo stream RGB para mostrar (sin tintes raros)
# - Reescalado a 300x300 (o el input de tu modelo) para inferencia en la TPU
# - Por defecto usa: ssd_mobilenet_v1_cup_edgetpu.tflite y labels.txt de esta carpeta

import argparse
import os
import cv2
import numpy as np
from picamera2 import Picamera2

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter, run_inference

# Tamaño de preview que verás en pantalla (puedes cambiarlo)
DISPLAY_SIZE = (640, 480)   # por ejemplo (1280, 720) o (1640, 1232) si tu sensor lo soporta

def main():
    # Usamos esta carpeta (donde guardarás este script)
    here = os.path.dirname(os.path.abspath(__file__))
    default_model = os.path.join(here, 'ssd_mobilenet_v1_cup_edgetpu.tflite')
    default_labels = os.path.join(here, 'labels.txt')

    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default=default_model, help='Ruta al modelo .tflite (EdgeTPU)')
    parser.add_argument('--labels', default=default_labels, help='Ruta al archivo labels.txt')
    parser.add_argument('--threshold', type=float, default=0.6, help='Confianza mínima para dibujar detecciones')
    parser.add_argument('--top_k', type=int, default=10, help='Máximo de detecciones a mostrar')
    args = parser.parse_args()

    print(f'-> Cargando modelo: {args.model}')
    print(f'-> Cargando labels: {args.labels}')
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = read_label_file(args.labels)

    in_w, in_h = input_size(interpreter)  # p.ej. 300x300 para SSD MobileNet v1
    print(f'-> Tamaño de entrada del modelo: {in_w}x{in_h}')

    # ---- Cámara CSI (Picamera2): stream RGB para mostrar ----
    picam2 = Picamera2()
    # Preview grande en RGB (para mostrar)
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": DISPLAY_SIZE}
    )
    picam2.configure(config)
    # Opcional: fija algunos controles si quieres una imagen más estable
    # picam2.set_controls({"AwbEnable": True, "AfMode": 0})  # cámaras CSI típicamente no tienen AF
    picam2.start()
    # ---------------------------------------------------------

    window = 'CUP detection (q para salir)'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, DISPLAY_SIZE[0], DISPLAY_SIZE[1])

    try:
        while True:
            # Frame en RGB (Picamera2 ya entrega RGB)
            rgb = picam2.capture_array()

            # Reescalado solo para la TPU, manteniendo RGB
            rgb_small = cv2.resize(rgb, (in_w, in_h), interpolation=cv2.INTER_LINEAR)
            if rgb_small.dtype != np.uint8:
                rgb_small = rgb_small.astype(np.uint8)

            run_inference(interpreter, rgb_small.tobytes())
            objs = get_objects(interpreter, args.threshold)[:args.top_k]

            draw_detections(rgb, (in_w, in_h), objs, labels)

            cv2.imshow(window, rgb)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

def draw_detections(img_rgb, inference_size, objs, labels):
    """Dibuja bbox/etiquetas sobre un frame RGB.
       Usamos VERDE/BLANCO (se ven bien tanto en RGB como en BGR)."""
    h, w, _ = img_rgb.shape
    sx, sy = w / inference_size[0], h / inference_size[1]

    for obj in objs:
        # Reescala bbox a las dimensiones del frame mostrado
        bbox = obj.bbox.scale(sx, sy)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        score = int(100 * obj.score)
        cls_name = labels.get(obj.id, str(obj.id))
        label = f'{cls_name} {score}%'

        # Caja y texto
        cv2.rectangle(img_rgb, (x0, y0), (x1, y1), (0, 255, 0), 2)           # verde
        cv2.putText(img_rgb, label, (x0, max(0, y0 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)       # blanco

if __name__ == '__main__':
    main()
