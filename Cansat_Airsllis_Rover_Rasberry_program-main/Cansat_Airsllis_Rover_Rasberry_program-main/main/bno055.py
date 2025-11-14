# bno055_sensor.py

import time
import math
import board
import busio
import adafruit_bno055


def quaternion_to_yaw(w, x, y, z):
    # Returns yaw (heading) in radians, tilt-compensated
    t0 = 2.0 * (w * z + x * y)
    t1 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t0, t1)
    return yaw


class BNO055:
    
    """
    Lectura completa del BNO055 en modo NDOF.
    Incluye corrección por orientación del montaje.
    """

    DEFAULT_DECLINATION = 11.15  # Reno, Nevada (2025)

    def __init__(self,
                 address: int = 0x28,
                 mount_offset: float = 0.0,
                 flipped_x: bool = False):
        """
        address: dirección I2C del BNO055
        mount_offset: desfase fijo del montaje, en grados
        flipped_x: True si el sensor está girado 180° sobre el eje X
        """
        # Inicializa I2C y el sensor
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c, address=address)

        # Usa cristal externo y arranca en modo NDOF
        self.sensor.enable_external_crystal = True
        time.sleep(0.1)
        
        self.sensor.mode = 0x0C
        time.sleep(0.025)
        
        
        # Guarda parámetros de montaje
        self.mount_offset = mount_offset
        self.flipped_x = False  # Sensor is now mounted normally
        self.sensor.mode = 0x0C
        time.sleep(0.1)

    def get_heading_quaternion_radians(self):
        """
        Returns the tilt-compensated heading (yaw) in radians using quaternion output.
        """
        q = self.sensor.quaternion  # (w, x, y, z)
        if q is None or any(v is None for v in q):
            return 0.0
        yaw = quaternion_to_yaw(*q)
        # Optionally subtract mount_offset (in radians)
        yaw -= math.radians(self.mount_offset)
        # Normalize to [-pi, pi]
        return math.atan2(math.sin(yaw), math.cos(yaw))

    def get_heading_quaternion(self):
        """
        Returns the tilt-compensated heading (yaw) in degrees using quaternion output.
        """
        yaw = self.get_heading_quaternion_radians()
        deg = math.degrees(yaw)
        # Normalize to [0, 360)
        return deg % 360
    

    def read(self):
        """
        Devuelve todas las lecturas en un diccionario.
        Aplica corrección de orientación si flipped_x=True.
        """
        data = {
            "acceleration": self.sensor.acceleration,        # (m/s^2)
            "magnetometer": self.sensor.magnetic,            # (uT)
            "gyroscope": self.sensor.gyro,                   # (rad/s)
            "linear_acceleration": self.sensor.linear_acceleration,
            "gravity": self.sensor.gravity,
            "euler": self.sensor.euler,                      # (yaw, roll, pitch)
            "quaternion": self.sensor.quaternion,            # (w, x, y, z)
            "temperature": self.sensor.temperature,
            "calibration": self.sensor.calibration_status    # (sys, accel, gyro, mag)
        }

        # --- Corrección si el sensor está girado 180° en X ---
        if self.flipped_x:
            if data["acceleration"]:
                ax, ay, az = data["acceleration"]
                if ax is not None and ay is not None and az is not None:
                    data["acceleration"] = (ax, -ay, -az)
                else:
                    data["acceleration"] = (None, None, None)

            if data["gyroscope"]:
                gx, gy, gz = data["gyroscope"]
                if gx is not None and gy is not None and gz is not None:
                    data["gyroscope"] = (gx, -gy, -gz)
                else:
                    data["gyroscope"] = (None, None, None)

            if data["magnetometer"]:
                mx, my, mz = data["magnetometer"]
                if mx is not None and my is not None and mz is not None:
                    data["magnetometer"] = (mx, -my, -mz)
                else:
                    data["magnetometer"] = (None, None, None)

            if data["euler"]:
                yaw, roll, pitch = data["euler"]
                if yaw is not None and roll is not None and pitch is not None:
                    data["euler"] = (yaw, -roll, -pitch)
                else:
                    data["euler"] = (None, None, None)

        return data

    def get_heading(self):
        """
        Devuelve el heading (yaw) en grados, corrigiendo con mount_offset.
        """
        euler = self.sensor.euler
        if euler is None or euler[0] is None:
            return None
        return (euler[0] - self.mount_offset) % 360

    def get_heading_radians(self):
        """
        Devuelve el heading en radianes.
        """
        euler = self.sensor.euler
        if euler is None or euler[0] is None:
           return 0.0  # Valor por defecto si no hay heading
        yaw = euler[0] - self.mount_offset
        return math.radians(yaw)
    def get_calibration_status(self):
        """
        Devuelve el estado de calibración como una tupla: (sys, gyro, accel, mag)
        """
        return self.sensor.calibration_status

if __name__ == "__main__":
    sensor = BNO055(mount_offset=0.0, flipped_x=False)
    while True:
        data = sensor.read()
        print("Heading quaternion:", sensor.get_heading_quaternion(), "°")
        print("Heading normal:", sensor.get_heading(), "°")
        print("Euler (yaw, roll, pitch):", data["euler"])
        print("Magnetometer:", data["magnetometer"])
        print("Gyroscope:", data["gyroscope"])
        print("Accelerometer:", data["acceleration"])
        print("Calibration:", data["calibration"])
        print("----")
        

