# payload_codec.py
import struct
from typing import Dict, Any, Iterable

# ------- CSV (texto) -------
def make_csv(values: Iterable[Any]) -> str:
    """
    Recibe una lista/iterable de valores ya truncados (como haces hoy)
    y devuelve un string CSV (igual a tu csv_payload actual).
    """
    return ",".join(str(v) for v in values)

# ------- BINARIO (compacto + versionado) -------
# Formato binario: little-endian
# I  : ts_ms        (uint32)              4 bytes
# 3i : lat_i, lon_i, alt_dm (int32 * 3) 12 bytes
# 3h : ax_mg, ay_mg, az_mg  (int16 * 3)  6 bytes
# 3h : yaw_cd, pitch_cd, roll_cd (int16*3) 6 bytes
# H  : vbat_mV      (uint16)              2 bytes
# B  : version      (uint8)               1 byte
# Total: 31 bytes
FMT = "<I 3i 3h 3h H B"
BIN_VERSION = 1

def pack_binary(payload: Dict[str, float]) -> bytes:
    """
    Empaca en binario con escalas.
    Espera un dict con llaves (ponle tus reales):
      ts_ms, lat, lon, alt_m, ax_g, ay_g, az_g, yaw_deg, pitch_deg, roll_deg, vbat_v
    Devuelve bytes listos para hexlify y AT+PSEND.
    """
    ts_ms      = int(payload["ts_ms"])
    lat_i      = int(payload["lat"]      * 1e5)
    lon_i      = int(payload["lon"]      * 1e5)
    alt_dm     = int(payload["alt_m"]    * 10)
    ax_mg      = int(payload["ax_g"]     * 1000)
    ay_mg      = int(payload["ay_g"]     * 1000)
    az_mg      = int(payload["az_g"]     * 1000)
    yaw_cd     = int(payload["yaw_deg"]  * 100)
    pitch_cd   = int(payload["pitch_deg"]* 100)
    roll_cd    = int(payload["roll_deg"] * 100)
    vbat_mV    = int(payload["vbat_v"]   * 1000)
    version    = BIN_VERSION
    return struct.pack(FMT, ts_ms, lat_i, lon_i, alt_dm,
                       ax_mg, ay_mg, az_mg,
                       yaw_cd, pitch_cd, roll_cd,
                       vbat_mV, version)

def unpack_binary(b: bytes) -> Dict[str, float]:
    """
    Para el receptor: bytes -> dict con floats legibles (espejo de pack_binary).
    """
    import math
    (ts_ms, lat_i, lon_i, alt_dm,
     ax_mg, ay_mg, az_mg,
     yaw_cd, pitch_cd, roll_cd,
     vbat_mV, version) = struct.unpack(FMT, b)
    return {
        "ts_ms":     ts_ms,
        "lat":       lat_i / 1e5,
        "lon":       lon_i / 1e5,
        "alt_m":     alt_dm / 10.0,
        "ax_g":      ax_mg / 1000.0,
        "ay_g":      ay_mg / 1000.0,
        "az_g":      az_mg / 1000.0,
        "yaw_deg":   yaw_cd / 100.0,
        "pitch_deg": pitch_cd / 100.0,
        "roll_deg":  roll_cd / 100.0,
        "vbat_v":    vbat_mV / 1000.0,
        "version":   version
    }
