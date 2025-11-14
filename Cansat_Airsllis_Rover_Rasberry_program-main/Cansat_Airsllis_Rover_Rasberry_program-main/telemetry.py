# telemetry.py
import time, math

def getp(d, path, default=None):
    cur = d
    for k in path.split("."):
        if isinstance(cur, dict) and k in cur:
            cur = cur[k]
        elif k.isdigit() and isinstance(cur, (list, tuple)):
            idx = int(k)
            cur = cur[idx] if 0 <= idx < len(cur) else default
        else:
            return default
    return cur

FIELDS = {
    "gps.lat":          "gps.latitude",
    "gps.lon":          "gps.longitude",
    "bme.pressure":     "environment.pressure",
    "bme.altitude":     "environment.altitude_bme280",
    "ina.voltage":      "ina226.voltage",
    "ina.current":      "ina226.current",
    "ina.power":        "ina226.power",
    "acc.x":            "bno055.linear_acceleration.0",
    "acc.y":            "bno055.linear_acceleration.1",
    "acc.z":            "bno055.linear_acceleration.2",
    "euler.yaw":        "bno055.euler.yaw",     # si existe en tus datos
    "battery.percent":  "ina226.battery_pct",   # si lo calculas tú
}

def make_record(sensors_data, task, epoch, extra=None):
    data = {k: getp(sensors_data, path) for k, path in FIELDS.items()}
    ax, ay, az = data.get("acc.x"), data.get("acc.y"), data.get("acc.z")
    if all(isinstance(v, (int, float)) for v in (ax, ay, az)):
        data["acc.mag"] = math.sqrt(ax*ax + ay*ay + az*az)
    rec = {"ts": time.time(), "task": task, "epoch": epoch, "data": data}
    if extra:
        rec.update(extra)
    return rec
