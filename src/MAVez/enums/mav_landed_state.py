# mav_landed_state.py
# version: 1.0.0
# Original Author: Theodore Tasman
# Creation Date: 2026-03-04
# Last Modified: 2025-03-04
# Organization: PSU UAS

from enum import Enum

class MAVLandedState(Enum):
    """Enum for MAVLink landed states.
    """
    MAV_LANDED_STATE_UNDEFINED = 0
    MAV_LANDED_STATE_ON_GROUND = 1
    MAV_LANDED_STATE_IN_AIR = 2
    MAV_LANDED_STATE_TAKEOFF = 3
    MAV_LANDED_STATE_LANDING = 4

    @staticmethod
    def string(state_code: int | None) -> str:
        """Get the MAV_LANDED_STATE string from the enum

        Args:
            state_code (int | None): Integer value of MAV_LANDED_STATE enum

        Returns:
            str: String representation of corresponding MAV_LANDED_STATE
        """
        if state_code is None:
            return "UNKNOWN"
        
        try:
            return MAVLandedState(state_code).name
        except ValueError:
            return f"UNKNOWN CODE: {state_code}"