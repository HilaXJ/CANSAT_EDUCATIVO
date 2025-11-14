import serial
import time

PORT = "/dev/ttyAMA1"   # ajusta si hace falta
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

def cmd(s, wait=0.15):
    ser.write((s + "\r\n").encode())
    time.sleep(wait)
    resp = ser.read_all().decode(errors="ignore")
    if resp.strip():
        print("[CMD RESP]", resp.strip())

# Configurar P2P TX
cmd("AT")
cmd("AT+NWM=0")                              # P2P LoRa
cmd("AT+P2P=915000000:7:125:0:10:14")        # mismos params
cmd("AT+PRECV=0")                            # asegurar modo TX

i = 0
print("Enviando paquetes P2P...\n")
while True:
    text = f"HOLA_{i}"
    payload_hex = text.encode().hex()

    cmd(f"AT+PSEND={payload_hex}", wait=0.3)  # manda y muestra respuesta
    i += 1
    time.sleep(2)
