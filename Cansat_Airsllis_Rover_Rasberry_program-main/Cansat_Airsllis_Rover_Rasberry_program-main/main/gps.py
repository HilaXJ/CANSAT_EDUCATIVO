
from serial import Serial
from sphericalTrigonometry import SphericalPoint
import time
import threading

class GPS:
    GGA_TYPE = 'GGA'

    def __init__(self, port="/dev/serial0", baud=9600, timeout=1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.serial=Serial(port, baud, timeout=timeout)
        self.last_point = SphericalPoint(0.0, 0.0)
        self.last_alt = 0.0
        self._stop_thread = False
        self.debug = False  # Cambia a True para ver mensajes de depuración
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _update_loop(self):
        while not self._stop_thread:
            try:
                point, alt = self.read()  # antes se pasaba blocking=True (param inexistente) => error silenciado
                # Actualizar sólo si parece un valor válido (evitar quedarse en 0,0 si todavía no hay fix)
                if (point.latitude != 0.0 or point.longitude != 0.0):
                    self.last_point = point
                    self.last_alt = alt
                    if self.debug:
                        print(f"[GPS] Update lat={point.latitude:.6f} lon={point.longitude:.6f} alt={alt:.1f}")
            except Exception as e:
                if self.debug:
                    print(f"[GPS] Error hilo: {e}")
            # No sleep: el GPS es lento por sí mismo

    def parse_nmea_sentence(self, sentence):
        parsed_sentence = {}
        values = sentence.split(',')
        parsed_sentence['type'] = values[0][3:] if len(values[0]) >= 6 else ""

        if parsed_sentence['type'] == self.GGA_TYPE and len(values) >= 10:
            # Latitude
            if values[2]:
                latitude = int(values[2][:2]) + float(values[2][2:]) / 60.0
                if values[3] == 'S':
                    latitude = -latitude
                parsed_sentence['latitude'] = latitude

            # Longitude
            if values[4]:
                longitude = int(values[4][:3]) + float(values[4][3:]) / 60.0
                if values[5] == 'W':
                    longitude = -longitude
                parsed_sentence['longitude'] = longitude

            # Altitude
            try:
                parsed_sentence['altitude'] = float(values[9]) if values[9] else None
            except ValueError:
                parsed_sentence['altitude'] = None

        return parsed_sentence

    def read(self):
        parsed_sentence = {}
        while True:
            try:
                data = self.serial.readline()
                sentence = data.decode('utf-8', errors='ignore').strip()
                if not sentence.startswith("$"):
                    continue

                parsed_sentence = self.parse_nmea_sentence(sentence)

                if parsed_sentence.get('type') == self.GGA_TYPE:
                    break
            except Exception as e:
                if self.debug:
                    print(f"[GPS] Error lectura: {e}")
                continue

        lat = parsed_sentence.get('latitude', 0.0)
        lon = parsed_sentence.get('longitude', 0.0)
        alt = parsed_sentence.get('altitude', 0.0) or 0.0
        # Escribir último punto (no crítico si falla)
        try:
            with open("gps_data.txt", "w") as f:
                f.write(f"{lat},{lon}\n")
        except Exception as e:
            if self.debug:
                pass#print(f"[GPS] Error escribiendo archivo: {e}")

        return SphericalPoint(lat, lon), alt

    def stop(self):
        self._stop_thread = True
        if self._thread.is_alive():
            self._thread.join()


if __name__ == '__main__':
    gps = GPS(port="/dev/serial0")
    gps.debug = True  # activar logs internos si quieres
    print("=== Test GPS (Ctrl+C para salir) ===")
    print("Mostrando última muestra válida actualizada por el hilo (puede tardar hasta tener fix)...")
    try:
        while True:
            lp = gps.last_point
            alt = gps.last_alt
            print(f"LAST -> lat={lp.latitude:.6f} lon={lp.longitude:.6f} alt={alt:.1f}")
            # Si quieres forzar una lectura inmediata adicional (bloqueante hasta GGA) descomenta:
            # p, a = gps.read()
            # print(f"READ -> lat={p.latitude:.6f} lon={p.longitude:.6f} alt={a:.1f}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nFin prueba GPS")
    finally:
        gps.stop()
          
