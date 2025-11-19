#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import collections
import os
import re
import time

import cv2
import numpy as np
from picamera2 import Picamera2

import common
from tracker import ObjectTracker

# ---------------------------------------------------------------------------
# Tipos auxiliares (igual que tu script original)
# ---------------------------------------------------------------------------

Object = collections.namedtuple('Object', ['id', 'score', 'bbox'])

class BBox(collections.namedtuple('BBox', ['xmin', 'ymin', 'xmax', 'ymax'])):
    __slots__ = ()

# Resultado "compacto" para usar en tu RoverManager
DetectionResult = collections.namedtuple(
    'DetectionResult',
    [
        'has_detection',   # bool
        'distance_m',      # float o None
        'offset_x_norm',   # float en [-1, 1] o None
        'bbox_px',         # (x0, y0, x1, y1) o None
        'score',           # confianza [0,1]
        'track_id',        # int o None
        'fps',             # float
        'inf_ms',          # float
        'timestamp',       # time.time()
    ]
)

# ---------------------------------------------------------------------------
# Funciones auxiliares (idénticas o casi a tu script)
# ---------------------------------------------------------------------------

def set_input_np(interpreter, image_rgb):
    """Carga imagen RGB (HxWx3) en el tensor de entrada del TFLite/EdgeTPU."""
    input_details = interpreter.get_input_details()[0]
    ih, iw = input_details['shape'][1], input_details['shape'][2]

    if (image_rgb.shape[1], image_rgb.shape[0]) != (iw, ih):
        image_rgb = cv2.resize(image_rgb, (iw, ih), interpolation=cv2.INTER_LINEAR)

    if input_details['dtype'] == np.uint8:
        image_rgb = image_rgb.astype(np.uint8)
    elif input_details['dtype'] == np.float32:
        image_rgb = (image_rgb.astype(np.float32) / 255.0)

    image_rgb = np.expand_dims(image_rgb, axis=0)
    interpreter.set_tensor(input_details['index'], image_rgb)

def load_labels(path):
    p = re.compile(r'\s*(\d+)(.+)')
    labels = {}
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            m = p.match(line)
            if m:
                idx, txt = m.groups()
                labels[int(idx)] = txt.strip()
    return labels

def get_detections(interpreter, threshold, top_k):
    """Lee salidas SSD EdgeTPU y devuelve Objects con bbox normalizado."""
    boxes = common.output_tensor(interpreter, 0)
    class_ids = common.output_tensor(interpreter, 1)
    scores = common.output_tensor(interpreter, 2)

    results = []
    for i in range(len(scores)):
        if scores[i] < threshold:
            continue
        if len(results) >= top_k:
            break
        ymin, xmin, ymax, xmax = boxes[i]
        bbox = BBox(
            xmin=float(np.clip(xmin, 0.0, 1.0)),
            ymin=float(np.clip(ymin, 0.0, 1.0)),
            xmax=float(np.clip(xmax, 0.0, 1.0)),
            ymax=float(np.clip(ymax, 0.0, 1.0)),
        )
        results.append(Object(
            id=int(class_ids[i]),
            score=float(scores[i]),
            bbox=bbox
        ))
    return results

def to_pixel_coords(bbox, w, h):
    return (
        int(bbox.xmin * w),
        int(bbox.ymin * h),
        int(bbox.xmax * w),
        int(bbox.ymax * h),
    )

# ---------------------------------------------------------------------------
# Clase principal: ConeDetector
# ---------------------------------------------------------------------------

class ConeDetector:
    """
    Envuelve toda la lógica de:
    - Picamera2
    - Coral USB (TFLite/EdgeTPU)
    - SORT (opcional)
    - Cálculo de distancia y error lateral normalizado

    Uso típico desde tu RoverManager:
        detector = ConeDetector(...)
        ...
        detection, frame = detector.process(draw=False)
        if detection.has_detection:
            Z = detection.distance_m
            ex = detection.offset_x_norm
            # usar Z y ex para el control de motores
    """

    def __init__(
        self,
        model_path='./models/ssd_mobilenet_v1_cone666_edgetpu.tflite',
        labels_path='./models/labels657.txt',
        threshold=0.6,
        top_k=10,
        cone_height_m=1.0,
        fov_v_deg=48.8,
        use_sort=True,
        avg_fps_window=60,
    ):
        self.threshold = float(threshold)
        self.top_k = int(top_k)
        self.cone_height_m = float(cone_height_m)
        self.fov_v_deg = float(fov_v_deg)

        print(f'[ConeDetector] Cargando modelo: {model_path}')
        self.interpreter = common.make_interpreter(model_path)
        self.interpreter.allocate_tensors()

        self.labels = load_labels(labels_path)

        # Tamaño de entrada del modelo
        in_w, in_h, _ = common.input_image_size(self.interpreter)
        self.src_w, self.src_h = in_w, in_h
        print(f'[ConeDetector] Tamaño entrada modelo: {self.src_w}x{self.src_h}')

        # Focal estimada en pixels (modelo pinhole simplificado)
        # fy = Hpx / (2 * tan(FOV_v/2))
        self.fy = self.src_h / (2.0 * np.tan(np.deg2rad(self.fov_v_deg / 2.0)))
        print(f'[ConeDetector] fy (px): {self.fy:.2f}')

        # Contador de FPS (de tu módulo common)
        self._fps_counter = common.avg_fps_counter(avg_fps_window)

        # Tracker SORT (opcional)
        self.mot_tracker = None
        if use_sort:
            try:
                obj_tracker = ObjectTracker('sort')
                self.mot_tracker = (
                    obj_tracker.trackerObject.mot_tracker
                    if obj_tracker.trackerObject else None
                )
                if self.mot_tracker is not None:
                    print('[ConeDetector] Tracker SORT habilitado')
                else:
                    print('[ConeDetector] No se pudo habilitar SORT (mot_tracker = None)')
            except Exception as e:
                print(f'[ConeDetector] Error inicializando SORT: {e}')
                self.mot_tracker = None
        else:
            print('[ConeDetector] SORT deshabilitado por configuración')

        # Cámara Picamera2 capturando al tamaño del modelo
        self.picam2 = Picamera2()
        cam_cfg = self.picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (self.src_w, self.src_h)}
        )
        self.picam2.configure(cam_cfg)
        self.picam2.start()
        time.sleep(0.5)
        print('[ConeDetector] Picamera2 inicializada')

        # Último resultado (útil si quieres consultarlo sin procesar un frame nuevo)
        self.last_result = DetectionResult(
            has_detection=False,
            distance_m=None,
            offset_x_norm=None,
            bbox_px=None,
            score=0.0,
            track_id=None,
            fps=0.0,
            inf_ms=0.0,
            timestamp=None,
        )

    # -----------------------------------------------------------------------
    # Método principal: procesa un frame y devuelve DetectionResult
    # -----------------------------------------------------------------------
    def process(self, draw=False):
        """
        Captura un frame, ejecuta la red + (opcionalmente) SORT y calcula:
        - distancia al cono (m)
        - offset lateral normalizado [-1,1]

        Parámetros
        ----------
        draw : bool
            Si True, dibuja bbox, texto de distancia, fps e inf_ms sobre frame.

        Returns
        -------
        result : DetectionResult
        frame  : np.ndarray o None
            Si draw=True, frame es el RGB con superposición para debug.
            Si draw=False, frame es None.
        """
        # 1) Captura
        frame_rgb = self.picam2.capture_array()

        # 2) Inferencia
        t0 = time.monotonic()
        set_input_np(self.interpreter, frame_rgb)
        self.interpreter.invoke()
        objs = get_detections(self.interpreter, self.threshold, self.top_k)
        t1 = time.monotonic()

        inf_ms = (t1 - t0) * 1000.0
        fps = next(self._fps_counter)

        # 3) Tracking (coordenadas normalizadas)
        tracks = []
        if self.mot_tracker is not None and objs:
            dets = np.array(
                [
                    [o.bbox.xmin, o.bbox.ymin, o.bbox.xmax, o.bbox.ymax, o.score]
                    for o in objs
                ],
                dtype=np.float32
            )
            tracks = self.mot_tracker.update(dets)

        # 4) Seleccionar mejor objetivo (para el control)
        best_distance = None
        best_offset = None
        best_bbox_px = None
        best_score = 0.0
        best_tid = None

        # --- Caso con SORT: usamos los tracks ---
        if len(tracks):
            best_area = -1.0
            for x1, y1, x2, y2, tid in tracks:
                px1 = int(x1 * self.src_w)
                py1 = int(y1 * self.src_h)
                px2 = int(x2 * self.src_w)
                py2 = int(y2 * self.src_h)

                w_px = max(1, px2 - px1)
                h_px = max(1, py2 - py1)
                area = w_px * h_px

                if area > best_area:
                    best_area = area
                    Z_m = (self.fy * self.cone_height_m) / float(h_px)

                    cx = (px1 + px2) / 2.0
                    img_cx = self.src_w / 2.0
                    e_x = cx - img_cx
                    offset_norm = e_x / (self.src_w / 2.0)

                    best_distance = Z_m
                    best_offset = offset_norm
                    best_bbox_px = (px1, py1, px2, py2)
                    best_tid = int(tid)
                    best_score = 1.0  # SORT no carga score, le ponemos 1.0

            if draw and best_bbox_px is not None:
                x0, y0, x1, y1 = best_bbox_px
                label = f'ID:{best_tid}  {best_distance:.2f} m'
                cv2.rectangle(frame_rgb, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2.putText(
                    frame_rgb, label, (x0, max(0, y0 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA
                )

        # --- Caso sin SORT: usamos la detección con mayor score ---
        elif objs:
            best = max(objs, key=lambda o: o.score)
            x0, y0, x1, y1 = to_pixel_coords(best.bbox, self.src_w, self.src_h)
            h_px = max(1, y1 - y0)
            Z_m = (self.fy * self.cone_height_m) / float(h_px)

            cx = (x0 + x1) / 2.0
            img_cx = self.src_w / 2.0
            e_x = cx - img_cx
            offset_norm = e_x / (self.src_w / 2.0)

            best_distance = Z_m
            best_offset = offset_norm
            best_bbox_px = (x0, y0, x1, y1)
            best_score = best.score
            best_tid = None

            if draw:
                pct = int(best.score * 100)
                name = self.labels.get(best.id, best.id)
                txt = f'{pct}% {name}  {Z_m:.2f} m'
                cv2.rectangle(frame_rgb, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2.putText(
                    frame_rgb, txt, (x0, max(0, y0 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA
                )

        # 5) Construir resultado
        has_det = best_distance is not None
        result = DetectionResult(
            has_detection=has_det,
            distance_m=best_distance,
            offset_x_norm=best_offset,
            bbox_px=best_bbox_px,
            score=best_score,
            track_id=best_tid,
            fps=fps,
            inf_ms=inf_ms,
            timestamp=time.time(),
        )

        self.last_result = result

        if draw:
            info = f'Inf: {inf_ms:.2f} ms  FPS: {fps:.0f}'
            cv2.putText(
                frame_rgb, info, (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA
            )
            return result, frame_rgb
        else:
            return result, None

    def get_last_result(self):
        """Devuelve el último DetectionResult calculado."""
        return self.last_result

    def close(self):
        """Libera recursos de cámara (llamar al cerrar la app)."""
        try:
            self.picam2.stop()
        except Exception:
            pass

# ---------------------------------------------------------------------------
# Bloque de prueba independiente (opcional)
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    detector = ConeDetector()
    window_name = 'ConeDetector debug (q para salir)'
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            result, frame = detector.process(draw=True)

            # Mostrar datos básicos por consola
            if result.has_detection:
                print(
                    f'Dist: {result.distance_m:.2f} m  '
                    f'offset_x_norm: {result.offset_x_norm:.3f}  '
                    f'fps: {result.fps:.1f}'
                )

            cv2.imshow(window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        detector.close()
        cv2.destroyAllWindows()
