#!/usr/bin/env python3
import serial
import time
import binascii
import os
import ast

PORT = "/dev/serial0"
BAUD = 115200
P2P = "915000000:7:0:0:16:20"  # Debe ser igual que en el emisor
OUTPUT_DB = "rx_database.csv"  # Archivo base de datos CSV

def at(ser, cmd, wait=0.25, show=True):
    ser.write((cmd + "\r\n").encode())
    ser.flush()
    time.sleep(wait)
    out = ser.read(ser.in_waiting or 1).decode(errors="ignore")
    time.sleep(0.08)
    out += ser.read(ser.in_waiting or 1).decode(errors="ignore")
    if show:
        print(f"> {cmd}\n{out.strip()}\n")
    return out

def rearm_rx(ser):
    at(ser, "AT+PRECV=0", show=False, wait=0.05)
    at(ser, "AT+PRECV=65535", show=False, wait=0.05)


def split_top_level(csv_line: str):
    """Divide una línea separada por comas ignorando comas dentro de paréntesis.
    Retorna lista de tokens crudos (strings sin trim final)."""
    out = []
    cur = []
    depth = 0
    for ch in csv_line:
        if ch == '(':
            depth += 1
            cur.append(ch)
        elif ch == ')':
            depth = max(0, depth - 1)
            cur.append(ch)
        elif ch == ',' and depth == 0:
            out.append(''.join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    if cur:
        out.append(''.join(cur).strip())
    return out


CSV_HEADER = [
    # Environment
    'env_temperature','env_pressure','env_humidity',
    # Battery
    'battery_voltage',
    # BNO055 vectors
    'accel_x','accel_y','accel_z',
    'mag_x','mag_y','mag_z',
    'gyro_x','gyro_y','gyro_z',
    'linacc_x','linacc_y','linacc_z',
    'grav_x','grav_y','grav_z',
    'euler_yaw','euler_roll','euler_pitch',
    'quat_w','quat_x','quat_y','quat_z',
    'bno_temp',
    'calib_sys','calib_accel','calib_gyro','calib_mag',
    # GPS
    'gps_latitude','gps_longitude','gps_altitude'
]

def ensure_header(path: str):
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        with open(path, 'w', encoding='utf-8', newline='') as f:
            f.write(','.join(CSV_HEADER) + '\n')

def flatten_record(record: dict):
    b = record['bno055']
    env = record['environment']
    gps = record['gps']
    def vec(v, n):
        v = list(v) if isinstance(v, (list, tuple)) else []
        return [str(v[i]) if i < len(v) and v[i] is not None else '' for i in range(n)]
    row = [
        env.get('temperature',''), env.get('pressure',''), env.get('humidity',''),
        record.get('battery_voltage','')
    ]
    row += vec(b.get('acceleration'),3)
    row += vec(b.get('magnetometer'),3)
    row += vec(b.get('gyroscope'),3)
    row += vec(b.get('linear_acceleration'),3)
    row += vec(b.get('gravity'),3)
    row += vec(b.get('euler'),3)
    row += vec(b.get('quaternion'),4)
    row.append(b.get('temperature',''))
    row += vec(b.get('calibration'),4)
    row += [gps.get('latitude',''), gps.get('longitude',''), gps.get('altitude_gps','')]
    return [str(x) for x in row]

def append_record(path: str, record: dict):
    ensure_header(path)
    line = ','.join(flatten_record(record))
    with open(path, 'a', encoding='utf-8', newline='') as f:
        f.write(line + '\n')


def parse_payload_to_record(payload: str):
    """Convierte la cadena recibida (última línea enviada) a un dict con la estructura solicitada.
    Se asume formato: temp,pressure,humidity,alt_baro,battery,accel,mag,gyro,lin_acc,gravity,euler,quat,bno_temp,calib,lat,lon,alt_gps
    Ignora alt_baro y no incluye encoders.
    """
    tokens = split_top_level(payload)
    # Validación mínima
    if len(tokens) < 17:
        raise ValueError(f"Payload insuficiente, tokens={len(tokens)}: {payload}")
    try:
        # Scalars
        temperature = float(tokens[0])
        pressure = float(tokens[1])
        humidity = float(tokens[2])
        # tokens[3] = alt_baro (ignorado)
        battery_voltage = float(tokens[4])

        def parse_tuple(tok):
            if tok.startswith('(') and tok.endswith(')'):
                return list(ast.literal_eval(tok))
            return []

        acceleration = parse_tuple(tokens[5])
        magnetometer = parse_tuple(tokens[6])
        gyroscope = parse_tuple(tokens[7])
        linear_acc = parse_tuple(tokens[8])
        gravity = parse_tuple(tokens[9])
        euler = parse_tuple(tokens[10])
        quaternion = parse_tuple(tokens[11])
        bno_temp = float(tokens[12]) if tokens[12] not in ('', 'None') else None
        calibration = parse_tuple(tokens[13])
        latitude = float(tokens[14])
        longitude = float(tokens[15])
        altitude_gps = float(tokens[16])
    except Exception as e:
        raise ValueError(f"Error parseando payload: {e}") from e

    record = {
        "environment": {
            "temperature": temperature,
            "pressure": pressure,
            "humidity": humidity
        },
        "battery_voltage": battery_voltage,
        "bno055": {
            "acceleration": acceleration,
            "magnetometer": magnetometer,
            "gyroscope": gyroscope,
            "linear_acceleration": linear_acc,
            "gravity": gravity,
            "euler": euler,
            "quaternion": quaternion,
            "temperature": bno_temp,
            "calibration": calibration
        },
        "gps": {
            "latitude": latitude,
            "longitude": longitude,
            "altitude_gps": altitude_gps
        }
    }
    return record

if __name__ == "__main__":
    # Reiniciar el archivo CSV y escribir encabezado
    with open(OUTPUT_DB, 'w', encoding='utf-8', newline='') as f:
        f.write(','.join(CSV_HEADER) + '\n')

    with serial.Serial(PORT, BAUD, timeout=0.15) as s:
        # Asegura que NO está en RX antes de configurar
        at(s, "AT+PRECV=0", show=False)
        at(s, "AT+NWM=0")
        at(s, f"AT+P2P={P2P}")

        # Entrar a escucha continua
        rearm_rx(s)
        print(">> Escuchando paquetes P2P ...\n")

        buf = ""
        while True:
            chunk = s.read(256).decode(errors="ignore")
            if not chunk:
                continue
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                print(line)
                if line.startswith("+EVT:RXP2P:"):
                    # +EVT:RXP2P:<RSSI>:<SNR>:<HEX>
                    parts = line.split(":")
                    hexpl = parts[-1] if len(parts) >= 4 else ""
                    if hexpl:
                        try:
                            txt = binascii.unhexlify(hexpl).decode("utf-8", errors="ignore")
                            if txt:
                                print("<< Texto:", txt)
                                # Intentar parsear y guardar
                                try:
                                    record = parse_payload_to_record(txt)
                                    append_record(OUTPUT_DB, record)
                                    print(f"<< Guardado en {OUTPUT_DB}")
                                except Exception as e:
                                    print(f"!! Error guardando payload: {e}")
                        except Exception:
                            pass
                    # re-armar recepción para el siguiente paquete
                    rearm_rx(s)
                elif "AT_BUSY_ERROR" in line or "P2P_RX_ON" in line:
                    rearm_rx(s)