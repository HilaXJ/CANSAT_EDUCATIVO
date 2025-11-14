import time
import board
import neopixel

# Usar GPIO21 (pin físico 40)
PIXEL_PIN = board.D21
NUM_PIXELS = 1

pixels = neopixel.NeoPixel(
    PIXEL_PIN,
    NUM_PIXELS,
    brightness=0.3,      # 0.0 a 1.0
    auto_write=True,
    pixel_order=neopixel.GRB
)

try:
    while True:
        # Rojo
        pixels[0] = (255, 0, 0)
        time.sleep(0.5)

        # Verde
        pixels[0] = (0, 255, 0)
        time.sleep(0.5)

        # Azul
        pixels[0] = (0, 0, 255)
        time.sleep(0.5)

        # Blanco
        pixels[0] = (255, 255, 255)
        time.sleep(0.5)

        # Apagado
        pixels[0] = (0, 0, 0)
        time.sleep(0.5)

except KeyboardInterrupt:
    pixels[0] = (0, 0, 0)
