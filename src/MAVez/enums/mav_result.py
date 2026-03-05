# mav_result.py
# version: 1.0.0
# Original Author: Theodore Tasman
# Creation Date: 2026-03-04
# Last Modified: 2025-03-04
# Organization: PSU UAS

from enum import Enum

class MAVResult(Enum):
    """Enum for MAVLink results.
    """
    MAV_RESULT_ACCEPTED = 0
    MAV_RESULT_TEMPORARILY_REJECTED = 1
    MAV_RESULT_DENIED = 2
    MAV_RESULT_UNSUPPORTED = 3
    MAV_RESULT_FAILED = 4
    MAV_RESULT_IN_PROGRESS = 5
    MAV_RESULT_CANCELLED = 6
    MAV_RESULT_COMMAND_LONG_ONLY = 7
    MAV_RESULT_COMMAND_INT_ONLY = 8
    MAV_RESULT_UNSUPPORTED_MAV_FRAME = 9

    @staticmethod
    def string(result_code: int | None) -> str:
        """Get the MAV_RESULT string from the enum

        Args:
            result_code (int | None): Integer value of MAV_RESULT enum

        Returns:
            str: String representation of corresponding MAV_RESULT
        """
        if result_code is None:
            return "UNKNOWN"
        
        try:
            return MAVResult(result_code).name
        except ValueError:
            return f"UNKNOWN CODE: {result_code}"