import json
import time
import threading
import queue
import logging
from logging.handlers import RotatingFileHandler


class IndustrialLogger:
    """
    Industrial-grade logger for robotics systems
    Features:
    - Non-blocking (queue-based)
    - JSON structured logs
    - Log levels
    - File rotation (SD-safe)
    - Network queue support
    """

    def __init__(self,
                 filename="client_log.json",
                 max_bytes=1_000_000,
                 backup_count=5):

        # Queues
        self.file_queue = queue.Queue()
        self.network_queue = queue.Queue()

        # Logger setup
        self.logger = logging.getLogger("IndustrialLogger")
        self.logger.setLevel(logging.INFO)

        handler = RotatingFileHandler(
            filename,
            maxBytes=max_bytes,
            backupCount=backup_count
        )

        formatter = logging.Formatter('%(message)s')
        handler.setFormatter(formatter)

        if not self.logger.handlers:
            self.logger.addHandler(handler)

        # Start background worker
        self.worker_thread = threading.Thread(
            target=self._process_logs,
            daemon=True
        )
        self.worker_thread.start()

    # ---------------- INTERNAL WORKER ----------------
    def _process_logs(self):
        while True:
            try:
                log_entry = self.file_queue.get()
                self.logger.info(json.dumps(log_entry))
            except Exception as e:
                print(f"Logger error: {e}")

    # ---------------- MAIN LOG FUNCTION ----------------
    def log(self, level, message, **kwargs):
        """
        Log structured message

        Example:
        logger.log("INFO", "Speed updated", speed=50)
        """

        log_entry = {
            "timestamp": time.time(),
            "level": level,
            "message": message,
            **kwargs
        }

        # Send to queues
        self.file_queue.put(log_entry)
        self.network_queue.put(log_entry)

        return log_entry

    # ---------------- CONVENIENCE METHODS ----------------
    def info(self, message, **kwargs):
        return self.log("INFO", message, **kwargs)

    def warning(self, message, **kwargs):
        return self.log("WARNING", message, **kwargs)

    def error(self, message, **kwargs):
        return self.log("ERROR", message, **kwargs)

    def critical(self, message, **kwargs):
        return self.log("CRITICAL", message, **kwargs)

    # ---------------- OPTIONAL: FLUSH ----------------
    def flush(self):
        """Wait until all logs are written"""
        while not self.file_queue.empty():
            time.sleep(0.01)