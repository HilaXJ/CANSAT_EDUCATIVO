from robot import Robot
from sphericalTrigonometry import SphericalPoint

class Calibration():
    def __init__(self,robot):
        self.bme280=robot.bme280
        self.ina226=robot.ina226
        self.gps=robot.gps
        self.bno055=robot.bno055
        self.left_encoder=robot.left_encoder
        self.right_encoder=robot.right_encoder

    def get_values(self):
        environment=self.bme280.read()#me devuelve un diccionario
        environment["altitude_bme280"]=self.bme280.get_altitude()
        battery_voltage=self.ina226.read_voltage()#me devuelve un valor
        bno055_data=self.bno055.read()#me devuelve un diccionario
        # GPS no bloqueante: usar la última muestra disponible del hilo
        latitude_longitude = getattr(self.gps, 'last_point', None)
        altitude_gps = getattr(self.gps, 'last_alt', 0.0)
        if latitude_longitude is None:
            latitude_longitude = SphericalPoint(0.0, 0.0)
        ticks_right=self.right_encoder.ticks()#me devuelve un valor
        ticks_left=self.left_encoder.ticks()#me devuelve un valor
        angle_right=self.right_encoder.angle_deg()#me devuelve un valor
        angle_left=self.left_encoder.angle_deg()#me devuelve un valor

        data = {
            "environment": environment,        # dict completo de BME280
            "battery_voltage": battery_voltage,
            "bno055": bno055_data,             # dict completo de BNO055
            "gps": {
                "latitude": latitude_longitude.latitude,
                "longitude": latitude_longitude.longitude,
                "altitude_gps": altitude_gps
            },
            "encoders": {
                "ticks_right": ticks_right,
                "ticks_left": ticks_left,
                "angle_right": angle_right,
                "angle_left": angle_left
            }
        }

        return data



