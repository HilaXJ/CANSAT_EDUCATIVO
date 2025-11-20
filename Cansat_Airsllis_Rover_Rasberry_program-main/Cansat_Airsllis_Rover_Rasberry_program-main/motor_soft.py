#!/usr/bin/env python3
import pigpio


class Motor:
    def __init__(self, in1_pin, in2_pin, freq=20000, pwm_range=100):
        self.in1 = in1_pin
        self.in2 = in2_pin
        self.FREQ = freq
        self.RANGE = pwm_range  # rango de duty para set_PWM_dutycycle (0..RANGE)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio no está corriendo. Ejecuta: sudo pigpiod")

        self.setup()
        self.stop()

    def setup(self):
        # Configurar pines como salida
        self.pi.set_mode(self.in1, pigpio.OUTPUT)
        self.pi.set_mode(self.in2, pigpio.OUTPUT)

        # Configurar frecuencia y rango de PWM por pin
        self.pi.set_PWM_frequency(self.in1, self.FREQ)
        self.pi.set_PWM_frequency(self.in2, self.FREQ)

        # Rango 0..RANGE (por defecto 0..100 → duty en % directo)
        self.pi.set_PWM_range(self.in1, self.RANGE)
        self.pi.set_PWM_range(self.in2, self.RANGE)

        # Arrancar con duty 0
        self.pi.set_PWM_dutycycle(self.in1, 0)
        self.pi.set_PWM_dutycycle(self.in2, 0)

    def stop(self):  # COAST
        """
        Motor en 'coast': ambos pines a 0, sin PWM.
        """
        # Duty 0 en ambos pines
        self.pi.set_PWM_dutycycle(self.in1, 0)
        self.pi.set_PWM_dutycycle(self.in2, 0)

        # Forzar salida en bajo
        self.pi.write(self.in1, 0)
        self.pi.write(self.in2, 0)

    def brake(self):
        """
        Freno activo: ambos pines a 1 (cortocircuito interno del motor).
        """
        # Quitar PWM (duty 0)
        self.pi.set_PWM_dutycycle(self.in1, 0)
        self.pi.set_PWM_dutycycle(self.in2, 0)

        # Ambos pines en alto
        self.pi.write(self.in1, 1)
        self.pi.write(self.in2, 1)

    def pwm(self, pin, duty_percent):
        """
        Aplica PWM 'soft' en un pin usando set_PWM_dutycycle.
        duty_percent: 0..100 (%)
        """
        duty_percent = max(0.0, min(100.0, float(duty_percent)))
        duty = int(self.RANGE * duty_percent / 100.0)  # escala a 0..RANGE
        self.pi.set_PWM_dutycycle(pin, duty)

    def forward(self, speed=100):
        """
        Sentido adelante: PWM en IN1, IN2 en 0.
        """
        # Asegurar que el pin contrario está apagado
        self.pi.set_PWM_dutycycle(self.in2, 0)
        self.pi.write(self.in2, 0)

        # PWM en IN1
        self.pwm(self.in1, speed)

    def backward(self, speed=100):
        """
        Sentido atrás: PWM en IN2, IN1 en 0.
        """
        # Asegurar que el pin contrario está apagado
        self.pi.set_PWM_dutycycle(self.in1, 0)
        self.pi.write(self.in1, 0)

        # PWM en IN2
        self.pwm(self.in2, speed)


if __name__ == '__main__':

    motor = Motor(13, 19)
    motor2 = Motor(18, 12)

    print("=== DRV8871 IN/IN (pigpio soft PWM) ===")
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
