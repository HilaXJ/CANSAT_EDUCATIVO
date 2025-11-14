#!/usr/bin/env python3
import serial, binascii
import time
import os
from gpiozero import OutputDevice
# --- Configuración RAK3172 ---
PORT = "/dev/serial0"
BAUD = 9600
P2P  = "915000000:7:0:0:16:20"   # Igual que en el receptor

# --- Funciones ---
def at(ser, cmd, wait=0.25, show=True):
    ser.write((cmd + "\r\n").encode())
    ser.flush()
    time.sleep(wait)
    return None  # No leer respuesta

def psend(ser, text):
    hexpl = binascii.hexlify(text.encode("utf-8")).decode()
    return at(ser, f"AT+PSEND={hexpl}", wait=0.4)
DATA_FILE = "data_to_send.txt"  # Archivo generado por main.py con datos ya truncados
# --- Main ---
with serial.Serial(PORT, BAUD, timeout=0.2) as s:
    LORA_RESET_PIN = 27  # Cambia este número si usas otro pin
    lora_reset = OutputDevice(LORA_RESET_PIN, active_high=True, initial_value=True)

    # Reset hardware LoRa
    lora_reset.off()
    time.sleep(0.5)
    lora_reset.on()
    time.sleep(0.1)

    # Configurar módulo en P2P
    at(s, "AT")
    at(s, "AT+PRECV=0", show=False)
    at(s, "AT+NWM=0")
    at(s, f"AT+P2P={P2P}")

    # Bucle principal: solo leer y enviar (sin reprocesar)
    while True:
        try:
            if os.path.exists(DATA_FILE):
                with open(DATA_FILE, 'r') as f:
                    lines = f.readlines()
                if lines:
                    payload = lines[-1].strip()
                    # Limitar longitud de seguridad (ajusta según SF si quieres)
                    max_len = 240
                    if len(payload) > max_len:
                        payload = payload[:max_len]
                    psend(s, payload)
                    print(f"Enviado: {payload}")
            time.sleep(1)
        except KeyboardInterrupt:
            break
