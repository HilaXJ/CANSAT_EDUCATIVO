import time
import sys
import qwiic_vl53l5cx

print("\n[VL53L5CX] Lectura simple\n")

tof = qwiic_vl53l5cx.QwiicVL53L5CX()

print(f"Direccion I2C: 0x{tof.address:02X}")

if not tof.is_connected():
    print("No se detecta el VL53L5CX en el bus I2C.", file=sys.stderr)
    sys.exit(1)

print("Inicializando (begin)...")
tof.begin()  # no devuelve True/False

# Opcional: 4x4 (16 zonas). Si falla internamente, igual queda con la resolución por defecto.
print("Configurando resolucion 4x4...")
tof.set_resolution(16)

print("Iniciando mediciones...")
tof.start_ranging()

try:
    while True:
        # Esperar a que haya datos nuevos
        if tof.check_data_ready():
            data = tof.get_ranging_data()
            if not data or not data.distance_mm:
                continue

            distances = data.distance_mm  # lista de distancias en mm
            n = len(distances)
            center_idx = n // 2
            center = distances[center_idx]
            d_min = min(distances)
            d_max = max(distances)

            print(f"Centro: {center:4} mm | Min: {d_min:4} mm | Max: {d_max:4} mm")
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nDeteniendo mediciones...")
    tof.stop_ranging()

