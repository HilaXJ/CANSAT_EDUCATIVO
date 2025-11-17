#!/usr/bin/env python3
import pigpio

class Motor:
    def __init__(self, in1_pin, in2_pin, freq=25000):
        self.in1 = in1_pin
        self.in2 = in2_pin
        self.FREQ = freq
        self.MAX_DUTY = 1_000_000

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio no está corriendo. Ejecuta: sudo pigpiod")

        self.setup()
        self.stop()

    def setup(self):
        self.pi.set_mode(self.in1, pigpio.OUTPUT)
        self.pi.set_mode(self.in2, pigpio.OUTPUT)

    def stop(self):  # COAST
        # Apagar completamente hardware PWM en ambos pines
        self.pi.hardware_PWM(self.in1, 0, 0)
        self.pi.hardware_PWM(self.in2, 0, 0)
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 0)

    def brake(self):
        # freno activo
        self.pi.hardware_PWM(self.in1, 0, 0)
        self.pi.hardware_PWM(self.in2, 0, 0)
        self.pi.write(self.in1, 1)
        self.pi.write(self.in2, 1)

    def pwm(self, pin, duty_percent):
        duty_percent = max(0.0, min(100.0, float(duty_percent)))
        duty = int(self.MAX_DUTY * duty_percent / 100.0)
        self.pi.hardware_PWM(pin, self.FREQ, duty)

    def forward(self, speed=100):
        # Apagar PWM en el pin contrario
        self.pi.hardware_PWM(self.in2, 0, 0)
        self.pi.write(self.in2, 0)

        # PWM en IN1
        self.pwm(self.in1, speed)

    def backward(self, speed=100):
        # Apagar PWM en el pin contrario
        self.pi.hardware_PWM(self.in1, 0, 0)
        self.pi.write(self.in1, 0)

        # PWM en IN2
        self.pwm(self.in2, speed)


if __name__ == '__main__':

    motor = Motor(13, 19)
    motor2 = Motor(18, 12)

    print("=== DRV8871 IN/IN ===")
    print("Comandos: f <duty%>, r <duty%>, s (stop), b (brake), q (salir)")
    print("Ejemplos: f 50   -> adelante 50%   |   r 80   -> atrás 80%")

    try:
        while True:
            cmd = input("> ").strip().split()
            if not cmd:
                continue

            c = cmd[0].lower()

            if c == "q":
                break

            elif c == "s":
                motor.stop()
                motor2.stop()
                print("→ STOP (coast)")

            elif c == "b":
                motor.brake()
                motor2.brake()
                print("→ BRAKE (freno activo)")

            elif c in ("f", "r") and len(cmd) == 2:
                try:
                    duty_percent = float(cmd[1])

                    if c == "f":
                        motor.forward(duty_percent)
                        motor2.forward(duty_percent)
                        print(f"→ Dir=adelante | Duty={duty_percent:.1f}%")
                    else:
                        motor.backward(duty_percent)
                        motor2.backward(duty_percent)
                        print(f"→ Dir=atrás | Duty={duty_percent:.1f}%")

                except ValueError:
                    print("Duty inválido. Usa un número entre 0 y 100.")

            else:
                print("Comando no válido.")

    finally:
        motor.stop()
        motor2.stop()
        print("Motores detenidos. Saliendo…")
