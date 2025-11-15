#!/usr/bin/env python3
import time
import queue

from lora_sender import LoRaP2PSender
from bme_280 import BME280Sensor   # ⬅️ tu driver del BME280

def main():
    # --- Inicializar BME280 ---
    bme = BME280Sensor()
    print("[BME280] Sensor inicializado.")

    # Cola para comunicarte con el hilo LoRa
    lora_queue = queue.Queue(maxsize=100)

    # Hilo emisor LoRa P2P (SIN reset pin)
    lora_thread = LoRaP2PSender(
        q=lora_queue,
        port="/dev/ttyAMA1",              # UART donde está tu RAK3172 TX
        baud=115200,                      # mismo baud que el módulo
        p2p="915000000:7:125:0:10:14",    # mismos parámetros P2P que el receptor
        period=1.0,                       # mínimo 1 s entre envíos
        max_len=240
    )

    lora_thread.start()
    print("[TEST] Hilo LoRaP2PSender iniciado. Enviando DATOS BME280...")
    print("       Ctrl+C para detener.\n")

    try:
        while True:
            # --- Leer BME280 ---
            env = bme.read()                  # dict: temp, pressure, humidity
            alt = bme.get_altitude()          # altitud calculada

            t  = env.get("temperature", 0.0)
            p  = env.get("pressure", 0.0)
            h  = env.get("humidity", 0.0)
            ts = time.time()

            # Payload compacto (cuidando longitud)
            # Formato: BME,<timestamp>,<T>,<P>,<H>,<ALT>
            payload = f"BME,{ts:.1f},{t:.2f},{p:.2f},{h:.2f},{alt:.2f}"

            # Mantener solo el último mensaje en la cola
            try:
                while True:
                    lora_queue.get_nowait()
            except queue.Empty:
                pass

            try:
                lora_queue.put_nowait(payload)
                print(f"[TEST] Encolado para envío: {payload}")
            except queue.Full:
                print("[WARN] Cola LoRa llena, se perdió un payload")

            time.sleep(1.0)   # una medición por segundo

    except KeyboardInterrupt:
        print("\n[TEST] Interrumpido por el usuario. Deteniendo hilo LoRa...")
    finally:
        try:
            lora_thread.stop()
            if lora_thread.is_alive():
                lora_thread.join(timeout=1.0)
        except Exception:
            pass
        print("[TEST] Fin de prueba LoRa.")

if __name__ == "__main__":
    main()
