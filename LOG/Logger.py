import json
import time
import threading
import queue
import logging
from logging.handlers import RotatingFileHandler

class IndustrialLogger:
    def __init__(self, filename="Client_Log.json", max_bytes=1_000_000, backup_count=5):
        self.file_queue = queue.Queue()
        self.network_queue = queue.Queue()

        self.logger = logging.getLogger(filename)
        self.logger.setLevel(logging.INFO)
        handler = RotatingFileHandler(filename, maxBytes=max_bytes, backupCount=backup_count)
        handler.setFormatter(logging.Formatter('%(message)s'))

        if not self.logger.handlers:
            self.logger.addHandler(handler)

        self.worker_thread = threading.Thread(target=self._process_logs, daemon=True)
        self.worker_thread.start()

    def _process_logs(self):
        while True:
            try:
                log_entry = self.file_queue.get()
                self.logger.info(json.dumps(log_entry))
            except Exception as e:
                print(f"[Logger] Error: {e}")

    def log(self, level, message, **kwargs):
        log_entry = {"timestamp": time.time(), "level": level, "message": message, **kwargs}
        self.file_queue.put(log_entry)
        self.network_queue.put(log_entry)
        return log_entry

    def info(self, message, **kwargs):
        return self.log("INFO", message, **kwargs)