
# ina226_sensor.py
#!/usr/bin/env python3
import time
try:
    from smbus import SMBus  # Bullseye
except ImportError:
    from smbus2 import SMBus

class INA226Sensor:
    """
    Encapsula el sensor INA226 para medición de voltaje del bus.
    """

    I2C_BUS = 1
    ADDR = 0x40       # dirección del INA226
    REG_BUS_V = 0x02  # Registro Bus Voltage (16 bits), LSB = 1.25 mV

    def __init__(self, i2c_addr: int = ADDR, bus: int = I2C_BUS):
        self.i2c_addr = i2c_addr
        self.bus = SMBus(bus)

    def _be16(self, x: int) -> int:
        """Convierte valor big-endian de 16 bits a little-endian."""
        return ((x & 0xFF) << 8) | (x >> 8)

    def read_voltage(self) -> float:
        """Retorna el voltaje de la batería en voltios."""
        raw = self._be16(self.bus.read_word_data(self.i2c_addr, self.REG_BUS_V)) & 0xFFFF
        return raw * 1.25e-3

    def close(self):
        self.bus.close()


if __name__ == "__main__":
    ina226 = INA226Sensor()
    try:
        while True:
            voltage = ina226.read_voltage()
            print(f"🔋 Battery Voltage: {voltage:.3f} V")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nTest finalizado por el usuario.")
    finally:
        ina226.close()

