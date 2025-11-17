from sphericalTrigonometry import SphericalPoint
from gps import GPS  # O el nombre real de tu archivo GPS
import time
import math

# --------------------------
# CONFIGURACIÓN DEL PUNTO DESTINO
# --------------------------
TARGET_LAT = -12.024609
TARGET_LON = -77.047370
TARGET = SphericalPoint(TARGET_LAT, TARGET_LON)

# --------------------------
# FUNCIÓN PARA NORMALIZAR HEADING (0–360°)
# --------------------------
def normalize_angle(angle_rad):
    angle_deg = math.degrees(angle_rad)
    angle_deg = (angle_deg + 360) % 360
    return angle_deg

# --------------------------
# MAIN: LEER GPS Y SACAR BEARING HACIA EL DESTINO
# --------------------------
def main():
    gps = GPS(port="/dev/serial0", baud=9600)
    print("=== BEARING hacia el punto guardado ===")
    
    time.sleep(1)

    try:
        while True:
            current = gps.last_point

            # Evitar cálculo si el GPS no tiene fix todavía
            if current.latitude == 0.0 and current.longitude == 0.0:
                print("Esperando señal GPS...")
                time.sleep(1)
                continue
            
            # Calcular bearing hacia destino
            theta_rad = current.bearingTo(TARGET)
            theta_deg = normalize_angle(theta_rad)

            print(f"\nGPS actual:")
            print(f"  Lat = {current.latitude:.6f}")
            print(f"  Lon = {current.longitude:.6f}")

            print("Ángulo hacia el punto objetivo:")
            print(f"  Bearing (rad) = {theta_rad:.3f}")
            print(f"  Bearing (deg) = {theta_deg:.1f}°")
            
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nCerrando programa...")
        gps.stop()

if __name__ == "__main__":
    main()
