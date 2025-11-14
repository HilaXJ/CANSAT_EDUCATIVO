
import math
import numpy as np
from bno055 import BNO055
from bme_280 import BME280Sensor
from ina226_sensor_1 import INA226Sensor as INA226Sensor_1
from ina226_sensor_2 import INA226Sensor as INA226Sensor_2
from motor import Motor
from encoder import QuadratureEncoder
from gps import GPS
import time
from sphericalTrigonometry import *
class Robot():
    def __init__(self, left_motor, right_motor, left_encoder, right_encoder, gps, bno055, bme280, ina226_1,ina226_2):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.speed = 0
        self.w = 0
        self.wheel_radius = 0.08
        self.wheel_base_length = 0.19
        self.max_left_wheel_speed = 10
        self.max_right_wheel_speed = 10
        self.max_speed = 10

        self.left_motor = left_motor
        self.right_motor = right_motor
        self.left_encoder = left_encoder
        self.right_encoder = right_encoder
        self.gps = gps
        self.bno055 = bno055
        self.bme280 = bme280
        self.ina226_1 = ina226_1
        self.ina226_2 = ina226_2  

        self.reference, _ = self.gps.read()
        self.theta = bno055.get_heading_radians()
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.stop()
    #ok
    def forward(self, speed = 1):
        self.left_motor.forward(speed)
        self.right_motor.forward(speed)
    #ok
    def backward(self, speed = 1):
        self.left_motor.backward(speed)
        self.right_motor.backward(speed)
    #ok
    def update_speed(self, left_wheel_speed, right_wheel_speed):#velocidad absoluta, este usan en maanger
        left_speed = left_wheel_speed / self.max_left_wheel_speed
        right_speed = right_wheel_speed / self.max_right_wheel_speed
        if left_speed < 0:
            self.left_motor.backward(-left_speed)
        else:
            self.left_motor.forward(left_speed)
        if right_speed < 0:
            self.right_motor.backward(-right_speed)
        else:
            self.right_motor.forward(right_speed)
    #ok
    def update_speed_normalize(self, left_speed, right_speed):#en porcentaje y ambos positivos
        if left_speed < 0:
            self.left_motor.backward(-left_speed)
        else:
            self.left_motor.forward(left_speed)
        if right_speed < 0:
            self.right_motor.backward(-right_speed)
        else:
            self.right_motor.forward(right_speed)

    def stop(self):#ok
        self.left_motor.stop()
        self.right_motor.stop()

    def get_state(self):#ok
        return np.array([
            [self.x],
            [self.y],
            [self.theta],
            [self.speed],
            [self.w]
        ])

    def get_environment(self):#ok
        return self.bme280.read()

    def get_battery_status(self):
        v1 = self.ina226_1.read_voltage()
        v2 = self.ina226_2.read_voltage()
        return {
            "battery_1": v1,
            "battery_2": v2
        }
    
    def active_calibration_bno055(self, timeout=120, speed=0.5):
        """
        Ejecuta un movimiento en forma de '8' hasta que el BNO055 esté calibrado.
        speed: velocidad normalizada (0.0 - 1.0)
        timeout: tiempo máximo de calibración en segundos
        """
        print("\n[CALIBRATION] Iniciando calibración activa del BNO055...")
        self.update_speed_normalize(speed,speed)
        time.sleep(5)
        self.update_speed_normalize(0,speed)
        time.sleep(5)
        self.update_speed_normalize(speed,0)
        time.sleep(3)
        self.update_speed_normalize(-speed,-speed)
        time.sleep(3)
        self.update_speed_normalize(speed,speed)
        time.sleep(1)
        self.update_speed_normalize(speed, speed*0.3)
        time.sleep(6)
        self.update_speed_normalize(0, 0)
        time.sleep(1)
        self.update_speed_normalize(speed*0.3, speed)
        time.sleep(6)
        self.update_speed_normalize(0, 0)
        time.sleep(1)
        t8_start = time.time()
        while time.time() - t8_start < 10:
            self.update_speed_normalize(speed, speed*0.3)
            time.sleep(2)
            self.update_speed_normalize(speed*0.3, speed)
            time.sleep(2)
        self.update_speed_normalize(0, 0)
        time.sleep(1)
        start_time = time.time()

        # Bucle principal de calibración
        while time.time() - start_time < timeout:
            sys, gyro, accel, mag = self.bno055.get_calibration_status()
            print(f"SYS={sys} GYRO={gyro} ACCEL={accel} MAG={mag}", end="\r")

            # Verificar si ya está completamente calibrado
            if sys >= 2 and gyro >= 3 and accel >= 0 and mag == 3:
                print("\n✅ BNO055 calibrado completamente durante el movimiento!")
                self.stop()
                return True

            # Movimiento en forma de 8: primero giro a la izquierda, luego a la derecha
            self.update_speed_normalize(speed,speed)
            time.sleep(5)
            self.update_speed_normalize(0,speed)
            time.sleep(5)
            self.update_speed_normalize(speed,0)
            time.sleep(3)
            self.update_speed_normalize(-speed,-speed)
            time.sleep(3)
            self.update_speed_normalize(speed,speed)
            time.sleep(1)
            self.update_speed_normalize(speed, speed*0.3)
            time.sleep(6)
            self.update_speed_normalize(0, 0)
            time.sleep(1)
            self.update_speed_normalize(speed*0.3, speed)
            time.sleep(6)
            self.update_speed_normalize(0, 0)
            time.sleep(1)
            t8_start = time.time()
            while time.time() - t8_start < 10:
                self.update_speed_normalize(speed, speed*0.3)
                time.sleep(2)
                self.update_speed_normalize(speed*0.3, speed)
                time.sleep(2)
            self.update_speed_normalize(0, 0)
            time.sleep(1)
            
            
        # Si no se calibró en el tiempo límite
        self.stop()
        print("\n⚠ Timeout: no se logró calibrar completamente el BNO055.")
        

    def get_full_state(self):#Ok
        ambiente=self.get_environment()
        gps_val=self.gps.read()
        battery = self.get_battery_status()
        return {
            "pose": {
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
                "speed": self.speed,
                "angular_speed": self.w
            },
            "temperature": ambiente["temperature"],
            "pressure": ambiente["pressure"],
            "humidity" : ambiente["humidity"],
            "battery": battery,
            "gps": { 
		"longitude":gps_val[0].longitude,
		"latitude":gps_val[0].latitude,
		"altitude":gps_val[1]
	    } 
        }
    #ok
    def set_state(self, state):
        self.x = state[0, 0]
        self.y = state[1, 0]
        self.theta = math.atan2(math.sin(state[2, 0]), math.cos(state[2, 0]))
        self.speed = state[3, 0]
        self.w = state[4, 0]
    def move_forward_with_heading(self, duration=10, speed=0.5):
        start_time = time.time()
        while time.time() - start_time < duration:
            self.left_motor.forward(speed)
            self.right_motor.forward(speed)
            heading = self.bno055.get_heading()
            print(f"Current heading: {heading:.2f}°")
            time.sleep(0.1)
        self.stop()

if __name__ == "__main__":

    # --- Initialize hardware ---
    left_motor = Motor(18,12)   # adjust pins!
    right_motor = Motor(13,19)

    left_encoder = QuadratureEncoder(tpr=985,
                                     pin_a=25, pin_b=24)
    right_encoder = QuadratureEncoder(tpr=985,
                                      pin_a=27, pin_b=17)

    gps = GPS()  # your GPS driver (or mock if not ready yet)

    bno055 = BNO055()
    bme280 = BME280Sensor()
    ina226_1 = INA226Sensor_1()
    ina226_2 = INA226Sensor_2()

    # --- Create robot instance ---
    robot = Robot(left_motor, right_motor,
                  left_encoder, right_encoder,
                  gps, bno055, bme280, ina226_1,ina226_2)
    """
    print("=== ROBOT TEST START ===")
    print("Initial state:", robot.get_full_state())

    # --- Test motors + encoders ---
    print("\n[TEST] Moving forward...")
    robot.left_motor.forward(50)
    robot.right_motor.forward(50)
    time.sleep(2)
    robot.stop()

    print("Left encoder ticks:", left_encoder.ticks())
    print("Right encoder ticks:", right_encoder.ticks())

    # --- Test sensors individually ---
    print("\n[TEST] Orientation (BNO055)")
    print("Heading [rad]:", robot.bno055.get_heading_radians())


    print("\n[TEST] Environment (BME280)")
    print(robot.get_environment())

    print("\n[TEST] Battery Voltage (INA226)")
    print(robot.get_battery_status())
    longitud_latitude, altitude = robot.gps.read()
    print("\n[TEST] GPS Position")
    print("longitude:",longitud_latitude.longitude)
    print("latitude:",longitud_latitude.latitude)
    print("altitude:",altitude)

    # --- Test backward movement ---
    print("\n[TEST] Moving backward...")
    robot.left_motor.backward(50)
    robot.right_motor.backward(50)
    time.sleep(2)
    robot.stop()

    print("Left encoder ticks:", left_encoder.ticks())
    print("Right encoder ticks:", right_encoder.ticks())

    print("\nFinal full state:", robot.get_full_state())
    print("=== ROBOT TEST END ===")
    """
    # ...inicialización de robot y sensores como ya tienes...

    # Simple test: move forward and print heading for 10 seconds
    robot.active_calibration_bno055()
    robot.move_forward_with_heading(duration=10, speed=1)
