#!/usr/bin/env python3
import pigpio
import time
import threading

PPR_ENCODER = 11
REDUCTION = 34.02
PPR = PPR_ENCODER*REDUCTION


##############################################################
# ===================== ENCODER ==============================
##############################################################

class Encoder:
    def __init__(self, pi, pinA, pinB):
        self.pi = pi
        self.pinA = pinA
        self.pinB = pinB
        
        self.position = 0

        self.pi.set_mode(pinA, pigpio.INPUT)
        self.pi.set_mode(pinB, pigpio.INPUT)

        self.pi.set_pull_up_down(pinA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(pinB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(pinA, pigpio.EITHER_EDGE, self._update)
        self.cbB = self.pi.callback(pinB, pigpio.EITHER_EDGE, self._update)

    def _update(self, pin, level, tick):
        A = self.pi.read(self.pinA)
        B = self.pi.read(self.pinB)

        if A == B:
            self.position += 1
        else:
            self.position -= 1

    def get_rpm(self, dt, ppr):
        pulses = self.position
        self.position = 0

        revs = pulses / ppr
        rpm = (revs / dt) * 60
        return rpm


##############################################################
# ======================= MOTOR ==============================
##############################################################

class Motor:
    def __init__(self, pi, in1, in2):
        self.pi = pi
        self.in1 = in1
        self.in2 = in2

        self.freq = 20000  # 20 kHz

        self.pi.set_mode(in1, pigpio.OUTPUT)
        self.pi.set_mode(in2, pigpio.OUTPUT)

    def set_speed(self, duty):   # duty: -100 → 100
        duty = max(-100, min(100, duty))
        dc = int(abs(duty) * 10000)  # pigpio: 0–1,000,000

        if duty > 0:
            self.pi.hardware_PWM(self.in1, self.freq, dc)
            self.pi.hardware_PWM(self.in2, self.freq, 0)

        elif duty < 0:
            self.pi.hardware_PWM(self.in1, self.freq, 0)
            self.pi.hardware_PWM(self.in2, self.freq, dc)

        else:
            self.stop()

    def stop(self):
        self.pi.hardware_PWM(self.in1, self.freq, 0)
        self.pi.hardware_PWM(self.in2, self.freq, 0)


##############################################################
# ========================= PID ==============================
##############################################################

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0
        self.last_error = 0

    def compute(self, setpoint, measured, dt):
        error = setpoint - measured
        self.integral += error * dt
        derivative = (error - self.last_error) / dt

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.last_error = error
        return output


##############################################################
# ===================== MAIN LOOP ============================
##############################################################

if __name__ == "__main__":
    pi = pigpio.pi()
    if not pi.connected:
        print("❌ pigpio no iniciado. Ejecuta: sudo pigpiod")
        exit()

    # ----- PINES DEL USUARIO ------
    M1_IN1 = 12
    M1_IN2 = 18
    EN1_A  = 27
    EN1_B  = 17

    M2_IN1 = 13
    M2_IN2 = 19
    EN2_A  = 25
    EN2_B  = 24

    # -------- OBJETOS -------------
    motor1 = Motor(pi, M1_IN1, M1_IN2)
    motor2 = Motor(pi, M2_IN1, M2_IN2)

    enc1 = Encoder(pi, EN1_A, EN1_B)
    enc2 = Encoder(pi, EN2_A, EN2_B)

    pid1 = PID(1.5, 0.2, 0.05)
    pid2 = PID(1.5, 0.2, 0.05)

    TARGET_RPM_1 = 80
    TARGET_RPM_2 = 80

    print("Running PID control...")
    last = time.time()

    while True:
        now = time.time()
        dt = now - last
        last = now

        rpm1 = enc1.get_rpm(dt, PPR)
        rpm2 = enc2.get_rpm(dt, PPR)

        duty1 = pid1.compute(TARGET_RPM_1, rpm1, dt)
        duty2 = pid2.compute(TARGET_RPM_2, rpm2, dt)

        motor1.set_speed(duty1)
        motor2.set_speed(duty2)

        print(f"M1: RPM={rpm1:.1f}, Duty={duty1:.1f} | "
              f"M2: RPM={rpm2:.1f}, Duty={duty2:.1f}")

        time.sleep(0.02)  # 50 Hz loop
