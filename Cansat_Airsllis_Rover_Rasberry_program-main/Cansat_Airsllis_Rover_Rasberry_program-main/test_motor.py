#!/usr/bin/env python3
import pigpio, time

IN1 = 13  # GPIO12
IN2 = 19   # GPIO18
FREQ = 1000
            # 20 kHz (inaudible)
MAX_DUTY = 1_000_000    # resolución de pigpio

class MotorININ:
    def __init__(self, in1, in2, freq=FREQ):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio no está corriendo. Ejecuta 'sudo pigpiod'.")
        self.in1, self.in2, self.freq = in1, in2, freq
        self.pi.set_mode(self.in1, pigpio.OUTPUT)
        self.pi.set_mode(self.in2, pigpio.OUTPUT)
        self.stop()

    def _pwm(self, pin, duty_pct):
        duty_pct = max(0.0, min(100.0, float(duty_pct)))
        duty = int(MAX_DUTY * duty_pct / 100.0)
        self.pi.hardware_PWM(pin, self.freq, duty)

    def stop(self):  # coast
        self.pi.hardware_PWM(self.in1, self.freq, 0)
        self.pi.hardware_PWM(self.in2, self.freq, 0)
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 0)

    # PWM en IN1, IN2 en LOW  -> un sentido
    def drive_in1(self, duty_pct):
        self.pi.write(self.in2, 0)              # asegurar IN2 en 0
        self._pwm(self.in1, duty_pct)           # PWM en IN1

    # PWM en IN2, IN1 en LOW  -> sentido inverso
    def drive_in2(self, duty_pct):
        self.pi.write(self.in1, 0)              # asegurar IN1 en 0
        self._pwm(self.in2, duty_pct)           # PWM en IN2

if __name__ == "__main__":
    m = MotorININ(IN1, IN2)
    print("Motor IN/IN (IN1=GPIO12, IN2=GPIO18)")
    print("Comandos: '1 <duty%>' PWM en IN1 | '2 <duty%>' PWM en IN2 | 's' stop | 'q' salir")
    print("Ejemplos: 1 30   (giro A 30%)   |   2 50   (giro B 50%)")
    try:
        while True:
            parts = input("> ").strip().split()
            if not parts: continue
            c = parts[0].lower()
            if c == "q":
                break
            elif c == "s":
                m.stop()
            elif c in ("1", "2") and len(parts) == 2:
                duty = float(parts[1])
                if c == "1":
                    m.drive_in1(duty)
                else:
                    m.drive_in2(duty)
            else:
                print("Uso: 1 <duty> | 2 <duty> | s | q")
    finally:
        m.stop()
