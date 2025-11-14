import qwiic_vl53l5cx
import sys
import time
from math import sqrt

def main():
    print("\nQwiic VL53L5CX - Distance Array\n")

    tof = qwiic_vl53l5cx.QwiicVL53L5CX()

    # 1. Verificar conexión
    if tof.is_connected() is False:
        print("El sensor no está conectado. Revisa Qwiic / I2C.", file=sys.stderr)
        sys.exit(1)

    print("Inicializando sensor. Esto puede tardar hasta ~10 s...")
    # 2. Inicializar
    if tof.begin() is False:
        print("Fallo en begin(). Saliendo.", file=sys.stderr)
        sys.exit(1)

    # 3. Habilitar todas las zonas (8x8). Es lo que usa SparkFun en el ejemplo.
    tof.set_resolution(8 * 8)

    # 4. Obtener resolución real desde el sensor
    image_resolution = tof.get_resolution()  # 16 o 64
    image_width = int(sqrt(image_resolution))

    # 5. Empezar mediciones
    tof.start_ranging()

    print("Leyendo distancias... (CTRL+C para salir)\n")

    try:
        while True:
            if tof.check_data_ready():
                measurement = tof.get_ranging_data()
                if not measurement or not measurement.distance_mm:
                    continue

                dist = measurement.distance_mm  # lista con image_resolution valores

                # Imprimir matriz (igual estilo SparkFun)
                for y in range(0, image_width * image_width, image_width):
                    fila = []
                    # recorremos de derecha a izquierda como el ejemplo
                    for x in range(image_width - 1, -1, -1):
                        fila.append(f"{dist[x + y]:4d}")
                    print(" ".join(fila))
                print()
                time.sleep(0.01)
            else:
                # pequeño delay para no saturar el bus si aún no hay dato
                time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nDeteniendo mediciones...")
        tof.stop_ranging()

if __name__ == "__main__":
    main()

