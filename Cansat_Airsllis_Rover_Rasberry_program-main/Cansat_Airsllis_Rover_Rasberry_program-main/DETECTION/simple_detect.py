# detect_picamera2_cone_min.py
# Picamera2 + PyCoral + OpenCV (versión mínima)

import os
import cv2
import numpy as np
from picamera2 import Picamera2
from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter, run_inference

# --- CONFIGURACIÓN BÁSICA (EDITA AQUÍ) ---
HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH  = os.path.join(HERE, 'ssd_mobilenet_v1_cone700_edgetpu.tflite')
LABELS_PATH = os.path.join(HERE, 'labels700.txt')
THRESHOLD   = 0.5
TOP_K       = 10
DISPLAY_W, DISPLAY_H = 640, 480
WINDOW_NAME = 'Cone detection (q para salir)'
# -----------------------------------------

def draw_boxes(img_rgb, objs, labels, in_size):
    """Dibuja bbox y etiqueta en coordenadas de la imagen mostrada."""
    h, w, _ = img_rgb.shape
    sx, sy = w / in_size[0], h / in_size[1]
    for o in objs:
        b = o.bbox.scale(sx, sy)
        x0, y0, x1, y1 = int(b.xmin), int(b.ymin), int(b.xmax), int(b.ymax)
        cls = labels.get(o.id, str(o.id))
        txt = f'{cls} {int(o.score*100)}%'
        cv2.rectangle(img_rgb, (x0, y0), (x1, y1), (0, 255, 0), 3)
        y_text = max(0, y0 - 10)
        cv2.putText(img_rgb, txt, (x0, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(img_rgb, txt, (x0, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)

def main():
    # Modelo y labels
    interpreter = make_interpreter(MODEL_PATH)
    interpreter.allocate_tensors()
    labels = read_label_file(LABELS_PATH)
    in_w, in_h = input_size(interpreter)  # p.ej. 300x300

    # Cámara
    picam2 = Picamera2()
    cfg = picam2.create_preview_configuration(main={"format": "RGB888",
                                                     "size": (DISPLAY_W, DISPLAY_H)})
    picam2.configure(cfg)
    picam2.start()

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            # Frame RGB desde el sensor
            rgb = picam2.capture_array()

            # Redimensionar solo para inferencia
            rgb_small = cv2.resize(rgb, (in_w, in_h), interpolation=cv2.INTER_LINEAR)
            if rgb_small.dtype != np.uint8:
                rgb_small = rgb_small.astype(np.uint8, copy=False)

            # Inferencia en EdgeTPU
            run_inference(interpreter, rgb_small.tobytes())
            objs = get_objects(interpreter, THRESHOLD)[:TOP_K]

            # Dibujar y mostrar
            draw_boxes(rgb, objs, labels, (in_w, in_h))
            cv2.imshow(WINDOW_NAME, rgb)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
