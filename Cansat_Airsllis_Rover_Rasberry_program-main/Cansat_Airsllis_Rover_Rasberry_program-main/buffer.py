# buzzer_simple_nosudo.py

from gpiozero import OutputDevice
import time

# Buzzer activo con transistor, nivel alto = suena
buzzer = OutputDevice(23, active_high=True, initial_value=False)

try:
    while True:
        buzzer.on()      # enciende
        time.sleep(0.2)
        buzzer.off()     # apaga
        time.sleep(0.8)
except KeyboardInterrupt:
    buzzer.off()
