# status_led.py

import time
import board
import neopixel


class StatusLed:
    """
    Manejo de un solo NeoPixel como LED de estado.
    """

    def __init__(self, pin=board.D21, brightness=0.3):
        self.pixels = neopixel.NeoPixel(
            pin,
            1,
            brightness=brightness,
            auto_write=True,
            pixel_order=neopixel.GRB
        )

    def set_color(self, r, g, b):
        self.pixels[0] = (r, g, b)

    def off(self):
        self.set_color(0, 0, 0)

    def close(self):
        # Opcional: libera el recurso
        try:
            self.off()
            self.pixels.deinit()
            print("Apagando y cerrando el LED...")
            pass

        except AttributeError:
            # Algunas versiones pueden no tener deinit()
            pass


# --- Clases de estado (un color / apagado) ---

class LedState:
    """Interfaz base para estados del LED."""
    def apply(self, led: StatusLed):
        raise NotImplementedError


class RedState(LedState):
    def apply(self, led: StatusLed):
        led.set_color(255, 0, 0)


class GreenState(LedState):
    def apply(self, led: StatusLed):
        led.set_color(0, 255, 0)


class BlueState(LedState):
    def apply(self, led: StatusLed):
        led.set_color(0, 0, 255)


class YellowState(LedState):
    def apply(self, led: StatusLed):
        # Rojo + Verde
        led.set_color(255, 255, 0)


class WhiteState(LedState):
    def apply(self, led: StatusLed):
        led.set_color(255, 255, 255)


class OffState(LedState):
    def apply(self, led: StatusLed):
        led.off()


def main():
    led = StatusLed(pin=board.D21, brightness=0.3)

    # 5 colores = 5 estados
    states = [
        RedState(),
        GreenState(),
        BlueState(),
        YellowState(),
        WhiteState(),
        OffState(),   # Apagado al final del ciclo
    ]

    try:
        while True:
            for state in states:
                state.apply(led)
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("Saliendo y apagando LED...")
        led.off()
        led.close()


if __name__ == "__main__":
    main()
