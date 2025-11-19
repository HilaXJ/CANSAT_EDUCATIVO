#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import threading


###########################################################
# ===================== ENCODER ===========================
###########################################################

class QuadratureEncoder:
    """
    Encoder incremental en modo cuadratura x2 (usa flancos de A).
    """
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
    """
    Control para DRV8871: un pin PWM, el otro en 0.
    """
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
        duty = max(0, min(100, duty))
        self.pwm2.stop()
        GPIO.output(self.in2, 0)
        self.pwm1.start(duty)

    def backward(self, duty):
        duty = max(0, min(100, duty))
        self.pwm1.stop()
        GPIO.output(self.in1, 0)
        self.pwm2.start(duty)


###########################################################
# ======================== MAIN ===========================
###########################################################

if __name__ == "__main__":
    motor1 = None
    motor2 = None

    try:
        print("Iniciando prueba de ENC + Motores M1 y M2...\n")

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Pines del Motor 1
        M1_IN1 = 12
        M1_IN2 = 18
        ENC1_A = 27
        ENC1_B = 17

        # Pines del Motor 2
        M2_IN1 = 13
        M2_IN2 = 19
        ENC2_A = 25
        ENC2_B = 24

        # TPR temporal — se calibrará luego con measure_tpr.py
        TPR = 1500

        # Crear motores
        motor1 = MotorDRV8871(M1_IN1, M1_IN2)
        motor2 = MotorDRV8871(M2_IN1, M2_IN2)

        # Crear encoders
        enc1 = QuadratureEncoder(ENC1_A, ENC1_B)
        enc2 = QuadratureEncoder(ENC2_A, ENC2_B)

        # Iniciar motores
        motor1.forward(100)
        motor2.forward(100)

        print("Motores girando al 50%. Ctrl+C para salir.\n")

        last = time.time()

        while True:
            time.sleep(0.1)
            now = time.time()
            dt = now - last
            last = now

            # Pulsos de cada motor
            p1 = enc1.read_and_reset()
            p2 = enc2.read_and_reset()

            # Calcular RPM
            rpm1 = (p1 / TPR) / dt * 60
            rpm2 = (p2 / TPR) / dt * 60

            print(
                f"M1: Pulsos={p1:4d}  RPM={rpm1:7.2f}   ||  "
                f"M2: Pulsos={p2:4d}  RPM={rpm2:7.2f}",
                end="\r"
            )

    except KeyboardInterrupt:
        print("\nSaliendo...")

    finally:
        if motor1: motor1.stop()
        if motor2: motor2.stop()
        GPIO.cleanup()
