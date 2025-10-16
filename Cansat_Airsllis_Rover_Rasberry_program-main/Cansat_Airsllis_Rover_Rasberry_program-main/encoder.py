

"""
import math

import RPi.GPIO as GPIO

class QuadratureEncoder():
    def __init__(self, ticks_per_revolution, hall_sensor_A, hall_sensor_B):

        self.counter = 0
        self.ticks_per_revolution = ticks_per_revolution
        self.hall_sensor_A = hall_sensor_A
        self.hall_sensor_B = hall_sensor_B
        self.previous_A = True
        self.previous_B = True

        self.setup()

    def setup(self):
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.hall_sensor_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.hall_sensor_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.hall_sensor_A, GPIO.BOTH, self.handle_edge)
        GPIO.add_event_detect(self.hall_sensor_B, GPIO.BOTH, self.handle_edge)

    def handle_edge(self, hall_sensor):
        current_A = GPIO.input(self.hall_sensor_A)
        current_B = GPIO.input(self.hall_sensor_B)

        turn = 0

        if hall_sensor == self.hall_sensor_A and current_A == 1:
            turn = 2 * self.previous_B - 1

        if hall_sensor == self.hall_sensor_B and current_B == 1:
            turn = 1 - 2 * self.previous_A

        self.previous_A = current_A
        self.previous_B = current_B

        self.counter += turn
"""
#!/usr/bin/env python3
import RPi.GPIO as GPIO
import threading
import time
class QuadratureEncoder:
    """
    Clase para leer un encoder incremental en modo cuadratura x2.
    Permite obtener los ticks acumulados y el ángulo en grados.
    """

    def __init__(self, pin_a: int, pin_b: int, tpr: int):
        """
        pin_a, pin_b: pines GPIO conectados al encoder.
        tpr: Ticks por revolución ya calculados (PPR * 2 * relación mecánica).
        """
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.tpr = max(1, int(tpr))  # evitar división por 0
        self._count = 0
        self._lock = threading.Lock()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._cb)

    def _cb(self, channel):
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        # Cuadratura x2: cuenta en ambos flancos de A. Dirección según B.
        delta = 1 if a == b else -1
        with self._lock:
            self._count += delta

    def ticks(self) -> int:
        """Retorna la cantidad acumulada de ticks."""
        with self._lock:
            return self._count

    def angle_deg(self) -> float:
        """Retorna el ángulo acumulado en grados (sin límite)."""
        return (self.ticks() * 360.0) / self.tpr
if __name__ == "__main__":
    try:
        # Configuración: pines GPIO del encoder y ticks por revolución
        pin_a = 5   #27 cambia según tu conexión
        pin_b = 6  #22 cambia según tu conexión
        tpr = 985    # ejemplo: 200 PPR * 2 (x2 cuadratura)

        encoder = QuadratureEncoder(pin_a, pin_b, tpr)

        print("=== Prueba de Encoder ===")
        print("Gira el eje y observa los ticks / ángulo...\n")

        while True:
            ticks = encoder.ticks()
            angle = encoder.angle_deg()
            print(f"Ticks: {ticks:6d} | Ángulo: {angle:8.2f}°", end="\r")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nSaliendo...")
    finally:
        GPIO.cleanup()
