#!/usr/bin/env python3
import time
import threading
import queue
import binascii
import serial
from gpiozero import OutputDevice

class LoRaP2PSender(threading.Thread):
    """
    Hilo que inicializa el RAK3172 en P2P y envía payloads por AT+PSEND (HEX).
    Usa una queue para recibir strings y las transmite sin bloquear el main.
    """

    def __init__(
        self,
        q: queue.Queue,
        port="/dev/serial0",
        baud=9600,
        p2p="915000000:7:0:0:16:20",
        reset_pin=None,        # AHORA PUEDE SER None
        period=1.0,
        max_len=240,
    ):
        super().__init__(daemon=True)
        self.q = q
        self.port = port
        self.baud = baud
        self.p2p = p2p
        self.reset_pin = reset_pin
        self.period = period
        self.max_len = max_len
        self._stop = False
        self.ser = None

    def stop(self):
        self._stop = True

    # --- helpers AT ---
    def _at(self, cmd, wait=0.25):
        self.ser.write((cmd + "\r\n").encode())
        self.ser.flush()
        time.sleep(wait)

    def _psend_text(self, text: str):
        if len(text) > self.max_len:
            text = text[:self.max_len]
        hexpl = binascii.hexlify(text.encode("utf-8")).decode()
        self._at(f"AT+PSEND={hexpl}", wait=0.4)

    def _reset_and_config(self):
        # Solo resetear si se especificó un pin
        if self.reset_pin is not None:
            rst = OutputDevice(self.reset_pin, active_high=True, initial_value=True)
            rst.off()
            time.sleep(0.5)
            rst.on()
            time.sleep(0.1)

        # Config LoRa P2P
        self._at("AT")
        self._at("AT+PRECV=0")        # asegurar modo TX
        self._at("AT+NWM=0")          # P2P
        self._at(f"AT+P2P={self.p2p}")

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self._reset_and_config()
            print("[LoRa] RAK3172 P2P listo.")
        except Exception as e:
            print(f"[LoRa] Init error: {e}")
            return

        last = 0.0
        while not self._stop:
            try:
                payload = None
                try:
                    payload = self.q.get(timeout=self.period)
                except queue.Empty:
                    payload = None

                if payload:
                    # pacing
                    dt = time.time() - last
                    if dt < self.period:
                        time.sleep(self.period - dt)
                    self._psend_text(payload.strip())
                    last = time.time()
                    print(f"[LoRa] Enviado: {payload.strip()}")

            except Exception as e:
                print(f"[LoRa] Error envío: {e}")
                time.sleep(0.5)

        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
