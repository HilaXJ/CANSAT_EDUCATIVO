#!/usr/bin/env python3
import time
import queue

from lora_sender import LoRaP2PSender
from motor import Motor
from encoder import QuadratureEncoder
from gps import GPS
from bno055 import BNO055
from bme_280 import BME280Sensor
from ina226_sensor_1 import INA226Sensor as INA226Sensor_1
from ina226_sensor_2 import INA226Sensor as INA226Sensor_2
from robot import Robot
from calibration import Calibration

from sphericalTrigonometry import SphericalPoint

class DummyGPS:
    """
    GPS falso para pruebas en interiores.
    Siempre devuelve lat=0.0, lon=0.0, alt=0.0
    y expone last_point / last_alt como haría el GPS real.
    """
    def __init__(self):
        self.last_point = SphericalPoint(0.0, 0.0)
        self.last_alt = 0.0

    def read(self):
        # Imitamos la interfaz del GPS real
        return self.last_point, self.last_alt



# ===== Helpers para aplanar y truncar (los mismos que en tu main) =====
def flatten_dict(d, parent_key='', sep='_'):
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep))
        else:
            items.append((new_key, v))
    return items

def truncate_value(key, value):
    # Lat / Lon a 5 decimales
    if isinstance(value, float):
        if any(x in key for x in ["latitude", "longitude"]):
            return round(value, 5)
        # cualquier otro float -> 1 decimal
        return round(value, 1)
    if isinstance(value, tuple):
        return tuple(round(v, 1) if isinstance(v, float) else v for v in value)
    return value

# ======================================================================

def main():
    # --------- Inicializar hardware + robot + calibration ----------
    left_motor = Motor(18, 12)
    right_motor = Motor(13, 19)

    left_encoder = QuadratureEncoder(tpr=985, pin_a=25, pin_b=24)
    right_encoder = QuadratureEncoder(tpr=985, pin_a=27, pin_b=17)

    gps = DummyGPS()
    bno055 = BNO055()
    bme280 = BME280Sensor()
    ina1 = INA226Sensor_1()
    ina2 = INA226Sensor_2()

    robot = Robot(left_motor, right_motor,
                  left_encoder, right_encoder,
                  gps, bno055, bme280, ina1, ina2)

    calibration = Calibration(robot)

    # --------- Hilo LoRa P2P ----------
    lora_queue = queue.Queue(maxsize=100)

    lora_thread = LoRaP2PSender(
        q=lora_queue,
        port="/dev/ttyAMA1",              # UART TX
        baud=115200,                      # baud del RAK TX
        p2p="915000000:7:125:0:10:14",    # mismos parámetros que el RX
        period=1.0,
        max_len=240
    )

    lora_thread.start()
    print("[TEST] LoRaP2PSender iniciado. Enviando payload igual que en misión...")
    print("       Ctrl+C para detener.\n")

    try:
        while True:
            # === 1) Leer todos los sensores como en misión ===
            sensors_data = calibration.get_values()

            # (Opcional) ver el diccionario completo en consola
            print("\n=== SENSORS_DATA ===")
            for k, v in sensors_data.items():
                print(k, ":", v)

            # === 2) Aplanar y filtrar encoders (igual que en tu main) ===
            flat = flatten_dict(sensors_data)
            filtered = [(k, v) for k, v in flat if 'encoders' not in k.lower()]
            values = [truncate_value(k, v) for k, v in filtered]
            str_values = [str(v) for v in values]
            csv_payload = ','.join(str_values)

            print(f"\n[PAYLOAD] {csv_payload}\n")

            # === 3) Encolar para LoRa: SOLO el último payload ===
            try:
                while True:
                    lora_queue.get_nowait()
            except queue.Empty:
                pass

            try:
                lora_queue.put_nowait(csv_payload)
                print("[TEST] Encolado para envío por LoRa.")
            except queue.Full:
                print("[WARN] Cola LoRa llena, se perdió un payload")

            time.sleep(1.0)  # misma cadencia que quieras en misión

    except KeyboardInterrupt:
        print("\n[TEST] Interrumpido por el usuario. Deteniendo hilo LoRa...")
    finally:
        try:
            lora_thread.stop()
            if lora_thread.is_alive():
                lora_thread.join(timeout=1.0)
        except Exception:
            pass
        robot.stop()
        print("[TEST] Fin de prueba LoRa con calibration.")

if __name__ == "__main__":
    main()
