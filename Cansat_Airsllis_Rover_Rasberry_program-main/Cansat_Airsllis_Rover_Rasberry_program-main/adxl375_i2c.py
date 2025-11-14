import smbus2
import time

class ADXL375:
    # Registros principales
    REG_DEVID       = 0x00
    REG_BW_RATE     = 0x2C
    REG_POWER_CTL   = 0x2D
    REG_DATA_FORMAT = 0x31
    REG_DATAX0      = 0x32  # X0, X1, Y0, Y1, Z0, Z1

    def __init__(self, bus: int = 1, address: int = 0x53):
        """
        address: 0x53 o 0x1D según ALT ADDRESS
        """
        self.bus_num = bus
        self.addr = address
        self.bus = smbus2.SMBus(bus)

        # Comprobar ID (debe ser 0xE5)
        devid = self.read_u8(self.REG_DEVID)
        if devid != 0xE5:
            print(f"[ADXL375] OJO: DEVID esperado 0xE5 y leo 0x{devid:02X}")

        # ODR ~100 Hz (0x0A) ver datasheet
        self.write_u8(self.REG_BW_RATE, 0x0A)

        # Formato datos: 0x0B = rango alto, justificado derecha (modo ADXL375)
        self.write_u8(self.REG_DATA_FORMAT, 0x0B)

        # Modo medida (bit Measure = 1)
        self.write_u8(self.REG_POWER_CTL, 0x08)
        time.sleep(0.01)

    def write_u8(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val & 0xFF)

    def read_u8(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def read_axes_raw(self):
        # Lee 6 bytes: X0,X1,Y0,Y1,Z0,Z1
        data = self.bus.read_i2c_block_data(self.addr, self.REG_DATAX0, 6)

        def to_int16(lo, hi):
            v = (hi << 8) | lo
            return v - 65536 if v & 0x8000 else v

        x = to_int16(data[0], data[1])
        y = to_int16(data[2], data[3])
        z = to_int16(data[4], data[5])
        return x, y, z

    def read_axes_g(self):
        x_raw, y_raw, z_raw = self.read_axes_raw()
        # ADXL375: ~49 mg/LSB = 0.049 g/LSB
        scale = 0.049
        return (
            x_raw * scale,
            y_raw * scale,
            z_raw * scale
        )

if __name__ == "__main__":
    sensor = ADXL375(address=0x53)  # cámbialo a 0x1D si tu i2cdetect lo muestra así
    try:
        while True:
            xg, yg, zg = sensor.read_axes_g()
            print(f"X={xg:7.2f} g  Y={yg:7.2f} g  Z={zg:7.2f} g")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nFin lectura ADXL375")
