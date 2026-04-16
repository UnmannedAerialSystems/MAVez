from enum import Enum

class RepositionLoiterMode(Enum):
    """Enum for MAVLink reposition loiter modes.
    """
    USE_YAW = -1
    CLOCKWISE = 0
    COUNTER_CLOCKWISE = 1

