# mav_mission_result.py
# version: 1.0.0
# Original Author: Theodore Tasman
# Creation Date: 2026-03-04
# Last Modified: 2025-03-04
# Organization: PSU UAS

from enum import Enum

class MAVMissionResult(Enum):
    """Enum for MAVLink mission results.
    """
    MAV_MISSION_ACCEPTED = 0
    MAV_MISSION_ERROR = 1
    MAV_MISSION_UNSUPPORTED_FRAME = 2
    MAV_MISSION_UNSUPPORTED = 3
    MAV_MISSION_NO_SPACE = 4
    MAV_MISSION_INVALID = 5
    MAV_MISSION_INVALID_PARAM1 = 6
    MAV_MISSION_INVALID_PARAM2 = 7
    MAV_MISSION_INVALID_PARAM3 = 8
    MAV_MISSION_INVALID_PARAM4 = 9
    MAV_MISSION_INVALID_PARAM5_X = 10
    MAV_MISSION_INVALID_PARAM6_Y = 11
    MAV_MISSION_INVALID_PARAM7_Z = 12
    MAV_MISSION_INVALID_SEQUENCE = 13
    MAV_MISSION_DENIED = 14
    MAV_MISSION_OPERATION_CANCELLED = 15

    @staticmethod
    def string(result_code: int | None) -> str:
        """Get the MAV_MISSION_RESULT string from the enum

        Args:
            result_code (int | None): Integer value of MAV_MISSION_RESULT enum

        Returns:
            str: String representation of corresponding MAV_MISSION_RESULT
        """
        if result_code is None:
            return "UNKNOWN"
        
        try:
            return MAVMissionResult(result_code).name
        except ValueError:
            return f"UNKNOWN CODE: {result_code}"