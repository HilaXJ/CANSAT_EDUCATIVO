class LogWriterThread(threading.Thread):
    """
    Escribe registros en NDJSON con rotación por tamaño.
    - Cada put() en la cola debe ser un dict JSON-serializable.
    - Rotación automática cuando el archivo supera rotate_mb.
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
        self._seq = 0
        self._f = None
        self._written = 0
        self._open_new_file()

    def stop(self):
        self._stop.set()

    def _now_stamp(self):
        return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    def _open_new_file(self):
        if self._f:
            try:
                self._f.flush(); os.fsync(self._f.fileno()); self._f.close()
            except Exception:
                pass
        fname = f"{self.basename}_{self._now_stamp()}_{self._seq:03d}.ndjson"
        self._seq += 1
        self._fpath = os.path.join(self.base_dir, fname)
        self._f = open(self._fpath, "a", buffering=1, encoding="utf-8")
        self._written = 0

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
        # último recurso: string
        return str(o)

    def run(self):
        pending = 0
        while not self._stop.is_set() or not self.q.empty():
            try:
                rec = self.q.get(timeout=0.5)
            except queue.Empty:
                # latido: flush para no perder datos si se apaga
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
                if pending >= self.flush_every:
                    self._f.flush()
                    try:
                        os.fsync(self._f.fileno())
                    except Exception:
                        pass
                    pending = 0
                if self._f.tell() >= self.rotate_bytes:
                    self._open_new_file()
            except Exception as e:
                # no detiene el hilo por un registro problemático
                try:
                    self._f.write(json.dumps({"logger_error": str(e)}) + "\n")
                except Exception:
                    pass
