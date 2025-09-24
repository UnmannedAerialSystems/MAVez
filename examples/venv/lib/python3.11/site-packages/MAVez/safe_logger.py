# safe_logger.py
# version: 1.0.0
# Original Author: Theodore Tasman
# Creation Date: 2025-09-15
# Last Modified: 2025-09-15
# Organization: PSU UAS

"""
A safe logger that only logs if the logger is properly initialized.
"""

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