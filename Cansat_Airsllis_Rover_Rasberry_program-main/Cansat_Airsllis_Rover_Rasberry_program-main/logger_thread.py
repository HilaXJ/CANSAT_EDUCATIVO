import os, json, datetime, threading, queue

class LogWriterThread(threading.Thread):
    """
    Logger NDJSON en segundo plano con rotación por tamaño.
    - Siempre crea un archivo nuevo al iniciar: ..._000.ndjson (o ..._001 si el 000 existe, etc.)
    - Mantiene un 'timestamp base' para que las rotaciones compartan el mismo prefijo.
    """
    def __init__(self, q: queue.Queue, base_dir: str, basename: str = "sensors",
                 rotate_mb: int = 20, flush_every: int = 25):
        super().__init__(daemon=True)
        self.q = q
        self.base_dir = base_dir
        self.basename = basename
        self.rotate_bytes = rotate_mb * 1024 * 1024
        self.flush_every = flush_every
        self._stop = threading.Event()      
        os.makedirs(self.base_dir, exist_ok=True)

        # --- NUEVO: timestamp "de grupo" fijo para esta corrida ---
        self._stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._seq = 0                  # siguiente sufijo a intentar
        self._f = None
        self._written = 0

        self._open_new_file()          # crea ..._000 o el siguiente libre

    def stop(self):
        self._stop.set()

    @staticmethod
    def _json_default(o):
        try:
            import numpy as np
            if isinstance(o, (np.floating,)):
                return float(o)
            if isinstance(o, (np.integer,)):
                return int(o)
            if isinstance(o, (np.ndarray,)):
                return o.tolist()
        except Exception:
            pass
        if isinstance(o, (set, tuple)):
            return list(o)
        return str(o)

    def _close_current(self):
        if self._f:
            try:
                self._f.flush()
                os.fsync(self._f.fileno())
                self._f.close()
            except Exception:
                pass
            self._f = None

    def _open_new_file(self):
        """Abre un nuevo archivo con sufijo incremental sin sobrescribir."""
        # Cierra el actual si existía
        self._close_current()

        # Intenta con el sufijo actual; si existe, incrementa hasta encontrar uno libre
        while True:
            fname = f"{self.basename}_{self._stamp}_{self._seq:03d}.ndjson"
            self._fpath = os.path.join(self.base_dir, fname)
            try:
                # 'x' = exclusive creation → falla si ya existe (evita append accidental)
                self._f = open(self._fpath, "x", buffering=1, encoding="utf-8")
                break
            except FileExistsError:
                self._seq += 1  # prueba el siguiente
                continue
        # Prepara el próximo sufijo para la siguiente rotación
        self._seq += 1
        self._written = 0

    def run(self):
        pending = 0
        while not self._stop.is_set() or not self.q.empty():
            try:
                rec = self.q.get(timeout=0.5)
            except queue.Empty:
                # latido: flush para minimizar pérdida ante apagado
                if self._f:
                    try:
                        self._f.flush()
                    except Exception:
                        pass
                continue

            try:
                line = json.dumps(rec, ensure_ascii=False, separators=(",", ":"), default=self._json_default)
                self._f.write(line + "\n")
                pending += 1

                # Flush + fsync por lotes
                if pending >= self.flush_every:
                    self._f.flush()
                    try:
                        os.fsync(self._f.fileno())
                    except Exception:
                        pass
                    pending = 0

                # Rotación por tamaño (conserva el mismo timestamp base y sube el sufijo)
                if self._f.tell() >= self.rotate_bytes:
                    self._open_new_file()

            except Exception as e:
                # No detiene el hilo si un registro falla; deja constancia
                try:
                    self._f.write(json.dumps({"logger_error": str(e)}) + "\n")
                except Exception:
                    pass

