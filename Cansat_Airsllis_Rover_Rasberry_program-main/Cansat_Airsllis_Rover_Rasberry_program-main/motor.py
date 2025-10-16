import pigpio

class Motor():
    def __init__(self, in1_pin, in2_pin):
        self.in1 = in1_pin
        self.in2 = in2_pin
        self.pi = pigpio.pi()
        self.FREQ = 20000
        self.MAX_DUTY = 1_000_000
        self.setup()
        self.stop()

    def setup(self):
        self.pi.set_mode(self.in1, pigpio.OUTPUT)
        self.pi.set_mode(self.in2, pigpio.OUTPUT)

    def stop(self):
        self.pi.hardware_PWM(self.in1, self.FREQ, 0)
        self.pi.hardware_PWM(self.in2, self.FREQ, 0)

    def pwm(self,pin, duty_percent):
        duty_percent = max(0.0, min(100.0, float(duty_percent)))
        duty = int(self.MAX_DUTY * duty_percent / 100.0)
        self.pi.hardware_PWM(pin, self.FREQ, duty)

    def forward(self, speed=100):#Velocidad den porcentaje(we will you these two)
        self.pi.hardware_PWM(self.in2,self.FREQ,0)
        self.pi.write(self.in2,0)
        self.pwm(self.in1,speed)   

    def backward(self, speed=100):#Velocidad den porcentaje
        self.pi.hardware_PWM(self.in1,self.FREQ,0)
        self.pi.write(self.in1,0)
        self.pwm(self.in2,speed)

if __name__ == '__main__':

    motor=Motor(13,19)
    motor2=Motor(18,12)
    print("=== DRV8871 IN/IN (IN1=GPIO18, IN2=GPIO12) ===")
    print("Comandos: f <duty%>, r <duty%>, s (stop/coast), b (brake), q (salir)")
    print("Ejemplos: f 50   -> adelante 50%   |   r 80   -> atrás 80%")

    try:
        while True:
            cmd = input("> ").strip().split()
            if not cmd:
                continue
            c = cmd[0].lower()

            if c == "q":
                break

            elif c in ("f", "r") and len(cmd) == 2:
                try:
                    duty_percent = float(cmd[1])
                    print(duty_percent)
                    if c == "f":
                        motor.forward(duty_percent)
                        motor2.forward(duty_percent)
                    else:
                        motor.backward(duty_percent)
                        motor2.backward(duty_percent)

                    print(f"→ Dir={'adelante' if c=='f' else 'atrás'} | Duty={duty_percent:.1f}%")
                except ValueError:
                    print("Duty inválido. Usa un número 0-100.")
            else:
                print("Comando no válido.")
    finally:
        motor.stop()
        motor.stop()

