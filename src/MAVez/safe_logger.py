# safe_logger.py
# version: 1.0.0
# Original Author: Theodore Tasman
# Creation Date: 2025-09-15
# Last Modified: 2025-09-15
# Organization: PSU UAS

"""
A safe logger that only logs if the logger is properly initialized.
"""

from datetime import datetime
import logging
import os
import colorlog


class SafeLogger:
    """
    A safe logger that only logs if the logger is properly initialized.

    Args:
        logger (logging.Logger | None): The logger instance to use. If None, logging is disabled.

    Returns:
        Safe_Logger: An instance of the Safe_Logger class.
    """

    def __init__(self, logger):
        self.logger = logger

    def debug(self, msg: str):
        if self.logger:
            self.logger.debug(msg)

    def info(self, msg: str):
        if self.logger:
            self.logger.info(msg)

    def warning(self, msg: str):
        if self.logger:
            self.logger.warning(msg)

    def error(self, msg: str):
        if self.logger:
            self.logger.error(msg)

    def critical(self, msg: str):
        if self.logger:
            self.logger.critical(msg)


def configure_logging():
    """
    Configure logging to log messages to both a file and the console with colors.
    """
    # Ensure the flight_logs directory exists
    os.makedirs("./flight_logs", exist_ok=True)

    # Create a log file with a timestamp
    log_filename = f"./flight_logs/log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.txt"

    # Get the logger
    logger = logging.getLogger()

    # Check if the logger already has handlers to avoid duplicates
    if not logger.handlers:
        # Configure the file handler
        file_handler = logging.FileHandler(log_filename)
        file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s\t- %(message)s'))

        # Configure the console handler with colors
        console_handler = colorlog.StreamHandler()
        console_handler.setFormatter(colorlog.ColoredFormatter(
            '%(log_color)s%(asctime)s - %(levelname)s - %(message)s',
            log_colors={
                'DEBUG': 'cyan',
                'INFO': 'green',
                'WARNING': 'yellow',
                'ERROR': 'red',
                'CRITICAL': 'bold_red',
            }
        ))

        # Add handlers to the logger
        logger.setLevel(logging.INFO)
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

    return logger