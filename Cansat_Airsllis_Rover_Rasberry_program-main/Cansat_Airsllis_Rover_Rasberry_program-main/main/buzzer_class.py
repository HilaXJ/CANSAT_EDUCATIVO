# buzzer_class.py

from gpiozero import OutputDevice
import time

class Buzzer:
    def __init__(self, pin=23, active_high=True):
        """
        Clase simple para manejar un buzzer con transistor.
        pin: GPIO que controla el buzzer (BCM).
        active_high: True si nivel alto lo enciende.
        """
        self.buzzer = OutputDevice(pin, active_high=active_high, initial_value=False)

    def beep(self, duration=2.0):
        """Hace sonar el buzzer durante 'duration' segundos."""
        self.buzzer.on()
        time.sleep(duration)
        self.buzzer.off()

    def close(self):
        """Libera el recurso del GPIO."""
        self.buzzer.close()


def main():
    # Prueba: sonar el buzzer 2 segundos y terminar
    buzz = Buzzer(pin=23)

    try:
        print("Buzzer sonando 2 segundos...")
        buzz.beep(2.0)   # aquí solo suena 2 s
        print("Listo.")
    finally:
        buzz.close()


if __name__ == "__main__":
    main()
