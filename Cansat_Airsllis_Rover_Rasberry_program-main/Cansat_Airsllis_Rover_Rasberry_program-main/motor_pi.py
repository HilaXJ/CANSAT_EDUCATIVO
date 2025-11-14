#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Puente MQTT en la Raspberry + control de relé por 'authorization':
- RX en TOPIC_FROM_WEB (app/tx). Si 'authorization' es true -> GPIO17 ON; false -> OFF
- TX en TOPIC_TO_WEB (app/rx)
- Entrada por teclado para enviar mensajes a la web
"""

import os, socket, time, json, ast, sys
from paho.mqtt import client as mqtt
from gpiozero import LED

# --- Configuración ---
BROKER_HOST = os.environ.get("MQTT_HOST", "localhost")
BROKER_PORT = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_USER   = os.environ.get("MQTT_USER")
MQTT_PASS   = os.environ.get("MQTT_PASS")

TOPIC_FROM_WEB = os.environ.get("TOPIC_FROM_WEB", "app/tx")  # Web -> Pi
TOPIC_TO_WEB   = os.environ.get("TOPIC_TO_WEB",   "app/rx")  # Pi -> Web
CLIENT_ID = f"pi-bridge-{socket.gethostname()}"

# GPIO del relé
RUN_PIN = int(os.environ.get("RUN_PIN", "17"))   # BCM 17
# Si tu módulo es ACTIVO-BAJO (muy común), deja 1; si es activo-alto pon 0
ACTIVE_LOW = os.environ.get("RELAY_ACTIVE_LOW", "1").lower() not in ("0", "false", "no")
relay = LED(RUN_PIN, active_high=not ACTIVE_LOW)   # relay.on() activa el contacto

def set_relay(state: bool):
    relay.on() if state else relay.off()

def parse_authorization(payload: str):
    """
    Devuelve True/False/None mirando varias formas:
      {"authorization": true}
      {"authorization": {"authorized": true}}
      {"authorized": true}
      "true"/"false" o "1"/"0"
      y fallback a repr de dict (ast.literal_eval)
    """
    s = payload.strip()
    obj = None
    try:
        obj = json.loads(s)
    except Exception:
        try:
            obj = ast.literal_eval(s)
        except Exception:
            obj = None

    if isinstance(obj, dict):
        if "authorization" in obj:
            a = obj["authorization"]
            if isinstance(a, dict) and "authorized" in a:
                return bool(a["authorized"])
            if isinstance(a, (bool, int)):
                return bool(a)
        if "authorized" in obj and isinstance(obj["authorized"], (bool, int)):
            return bool(obj["authorized"])

    sl = s.lower()
    if sl in ("true", "1", "on"):   return True
    if sl in ("false", "0", "off"): return False
    return None

# --- MQTT ---
def make_client():
    if hasattr(mqtt, "CallbackAPIVersion"):
        c = mqtt.Client(client_id=CLIENT_ID,
                        callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
    else:
        c = mqtt.Client(client_id=CLIENT_ID)
    if MQTT_USER:
        c.username_pw_set(MQTT_USER, MQTT_PASS)
    c.will_set(TOPIC_TO_WEB, f"[{CLIENT_ID}] offline", qos=0, retain=False)
    c.reconnect_delay_set(min_delay=1, max_delay=30)
    return c

client = make_client()

def on_connect(c, u, flags, rc):
    print(f"[MQTT] Conectado rc={rc} -> suscribiendo a '{TOPIC_FROM_WEB}'")
    c.subscribe(TOPIC_FROM_WEB)
    c.publish(TOPIC_TO_WEB, f"[{CLIENT_ID}] online", retain=True)

def on_message(c, u, msg):
    payload = msg.payload.decode(errors="ignore")
    print(f"[RX] {msg.topic}: {payload}")

    if payload.strip().lower() == "ping":
        c.publish(TOPIC_TO_WEB, "pong")
        return

    auth = parse_authorization(payload)
    if auth is not None:
        set_relay(auth)
        c.publish(TOPIC_TO_WEB, f"[{CLIENT_ID}] relay={'ON' if auth else 'OFF'}",
                  qos=1, retain=True)
    else:
        c.publish(TOPIC_TO_WEB, f"Pi recibio: {payload}")

def on_disconnect(c, u, rc):
    print(f"[MQTT] Desconectado rc={rc}")

client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

def main():
    print(f"Broker {BROKER_HOST}:{BROKER_PORT} | RX '{TOPIC_FROM_WEB}' | TX '{TOPIC_TO_WEB}'")
    set_relay(False)  # arranque en OFF
    client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
    client.loop_start()
    try:
        print("Escribe mensajes para enviarlos a la WEB (topic TX). Ctrl+C para salir.")
        while True:
            text = input("> ")
            if not text:
                continue
            # atajo local: 1/0 mueven el relé y avisan
            if text.strip() in ("1", "0"):
                st = text.strip() == "1"
                set_relay(st)
                client.publish(TOPIC_TO_WEB, f"[{CLIENT_ID}] relay={'ON' if st else 'OFF'} (CLI)",
                               qos=1, retain=True)
            else:
                client.publish(TOPIC_TO_WEB, text)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        set_relay(False)
        client.publish(TOPIC_TO_WEB, f"[{CLIENT_ID}] adios", retain=False)
        time.sleep(0.2)
        client.loop_stop()
        client.disconnect()
        print("Cerrado.")

if __name__ == "__main__":
    main()
