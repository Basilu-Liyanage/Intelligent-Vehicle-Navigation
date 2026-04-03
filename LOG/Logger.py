import json
import time
import threading
import queue
import logging
from logging.handlers import RotatingFileHandler


class IndustrialLogger:
    def __init__(self, filename="client_log.json"):
        self.file_queue = queue.Queue()
        self.network_queue = queue.Queue()

        self.log_id = 0  # 🔥 UNIQUE ID FOR SYNC

        self.logger = logging.getLogger("IndustrialLogger")
        self.logger.setLevel(logging.INFO)

        handler = RotatingFileHandler(filename, maxBytes=1_000_000, backupCount=5)
        handler.setFormatter(logging.Formatter('%(message)s'))

        if not self.logger.handlers:
            self.logger.addHandler(handler)

        threading.Thread(target=self._writer, daemon=True).start()

    def _writer(self):
        while True:
            log = self.file_queue.get()
            self.logger.info(json.dumps(log))

    def log(self, level, message, **kwargs):
        self.log_id += 1

        entry = {
            "id": self.log_id,
            "timestamp": time.time(),
            "level": level,
            "message": message,
            **kwargs
        }

        self.file_queue.put(entry)
        self.network_queue.put(entry)

        return entry

    def info(self, msg, **k): return self.log("INFO", msg, **k)
    def warning(self, msg, **k): return self.log("WARNING", msg, **k)
    def error(self, msg, **k): return self.log("ERROR", msg, **k)