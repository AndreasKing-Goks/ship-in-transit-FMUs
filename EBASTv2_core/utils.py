from pathlib import Path
import logging
import os
import psutil


class MemoryLogger:
    def __init__(
        self,
        log_dir,
        filename="memory_profile.log",
        logger_name="MemoryLogger"
    ):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.log_file = self.log_dir / filename

        self.logger = logging.getLogger(logger_name)
        self.logger.setLevel(logging.INFO)

        # Prevent duplicate handlers if class is created multiple times
        if not self.logger.handlers:

            file_handler = logging.FileHandler(
                self.log_file,
                mode="a",
                encoding="utf-8"
            )

            formatter = logging.Formatter(
                "%(asctime)s | %(message)s"
            )

            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

    def report_ram(self, tag):
        process = psutil.Process(os.getpid())
        ram_gb = process.memory_info().rss / (1024 ** 3)

        self.logger.info(
            f"{tag} | RAM={ram_gb:.3f} GB"
        )

        # Immediately flush to disk
        for handler in self.logger.handlers:
            handler.flush()

    def get_log_path(self):
        return str(self.log_file.resolve())