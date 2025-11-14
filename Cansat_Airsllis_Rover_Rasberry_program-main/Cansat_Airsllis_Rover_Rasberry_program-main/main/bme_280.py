
# bme280.py
import smbus2
import bme280
import time
import math
###########
class BME280Sensor:

    def __init__(self, i2c_addr: int = 0x76, bus: int = 1, sea_level_pressure: float = 1013.25, alpha: float = 0.1):
        
        self.bus = smbus2.SMBus(bus)
        self.i2c_addr = i2c_addr
        self.calibration_params = bme280.load_calibration_params(self.bus, self.i2c_addr)
        self.sea_level_pressure = sea_level_pressure
        self.alpha = alpha
        self._filtered_altitude = None  

    def read(self):
        data = bme280.sample(self.bus, self.i2c_addr, self.calibration_params)
        return {
            "temperature": data.temperature,
            "pressure": data.pressure,
            "humidity": data.humidity
        }

    def get_altitude(self):
        readings = self.read()
        pressure = readings["pressure"]

        # Fórmula barométrica
        raw_altitude = 44330 * (1 - (pressure / self.sea_level_pressure) ** (1/5.255))

        # Filtro exponencial
        if self._filtered_altitude is None:
            self._filtered_altitude = raw_altitude
        else:
            self._filtered_altitude = self.alpha * raw_altitude + (1 - self.alpha) * self._filtered_altitude

        return self._filtered_altitude

if __name__ == '__main__':
    try:
        bme280_sensor = BME280Sensor()
        while True:
            readings = bme280_sensor.read()
            altitude = bme280_sensor.get_altitude()
            print("🌡 Temp: {:.2f} °C | 💨 Hum: {:.2f} % | ⬇ Pressure: {:.2f} hPa | 🗻 Alt: {:.2f} m".format(
                readings["temperature"],
                readings["humidity"],
                readings["pressure"],
                altitude
            ))
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nTest finalizado por el usuario.")

