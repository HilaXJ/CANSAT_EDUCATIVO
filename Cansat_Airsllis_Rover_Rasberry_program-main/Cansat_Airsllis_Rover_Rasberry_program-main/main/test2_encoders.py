#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import threading

###########################################################
# ===================== ENCODER ===========================
###########################################################

class QuadratureEncoder:
    def __init__(self, pin_a, pin_b, name=""):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name

        self._count = 0
        self._lock = threading.Lock()

        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._cb)

    def _cb(self, channel):
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        delta = 1 if a == b else -1

        with self._lock:
            self._count += delta

    def read_and_reset(self):
        with self._lock:
            val = self._count
            self._count = 0
        return val


###########################################################
# ======================= MOTOR ===========================
###########################################################

class MotorDRV8871:
    def __init__(self, in1, in2, freq=20000):
        self.in1 = in1
        self.in2 = in2
        self.freq = freq

        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)

        self.pwm1 = GPIO.PWM(self.in1, freq)
        self.pwm2 = GPIO.PWM(self.in2, freq)

        self.stop()

    def stop(self):
        self.pwm1.stop()
        self.pwm2.stop()
        GPIO.output(self.in1, 0)
        GPIO.output(self.in2, 0)

    def forward(self, duty):
        self.pwm2.stop()
        GPIO.output(self.in2, 0)
        self.pwm1.start(duty)

    def backward(self, duty):
        self.pwm1.stop()
        GPIO.output(self.in1, 0)
        self.pwm2.start(duty)


###########################################################
# ======================== MAIN ===========================
###########################################################

if __name__ == "__main__":
    motor = None  # para evitar NameError en finally

    try:
        print("Iniciando prueba con RPi.GPIO + Encoder x2...\n")

        # IMPORTANTÍSIMO: configurar modo antes de usar GPIO.setup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Pines del motor
        M1_IN1 = 12
        M1_IN2 = 18

        # Pines del encoder
        ENC_A = 27
        ENC_B = 17

        TPR = 1500   # temporal, será calibrado

        motor = MotorDRV8871(M1_IN1, M1_IN2)
        enc = QuadratureEncoder(ENC_A, ENC_B)

        motor.forward(50)
        print("Motor girando a 50%. Ctrl+C para salir.\n")

        last = time.time()

        while True:
            time.sleep(0.1)
            now = time.time()
            dt = now - last
            last = now

            ticks = enc.read_and_reset()
            revs = ticks / TPR
            rpm = (revs / dt) * 60

            print(f"Ticks={ticks:5d} | RPM={rpm:7.2f}", end="\r")

    except KeyboardInterrupt:
        print("\nSaliendo...")

    finally:
        if motor is not None:
            motor.stop()
        GPIO.cleanup()
