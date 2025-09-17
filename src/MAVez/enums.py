# safe_logger.py
# version: 1.0.0
# Original Author: Theodore Tasman
# Creation Date: 2025-09-17
# Last Modified: 2025-09-17
# Organization: PSU UAS

MAV_RESULTS = {
    0: "MAV_RESULT_ACCEPTED",
    1: "MAV_RESULT_TEMPORARILY_REJECTED",
    2: "MAV_RESULT_DENIED",
    3: "MAV_RESULT_UNSUPPORTED",
    4: "MAV_RESULT_FAILED",
    5: "MAV_RESULT_IN_PROGRESS",
    6: "MAV_RESULT_CANCELLED",
    7: "MAV_RESULT_COMMAND_LONG_ONLY",
    8: "MAV_RESULT_COMMAND_INT_ONLY",
    9: "MAV_RESULT_UNSUPPORTED_MAV_FRAME"
}

MAV_MISSION_RESULT = {
    0: "MAV_MISSION_ACCEPTED",
    1: "MAV_MISSION_ERROR",
    2: "MAV_MISSION_UNSUPPORTED_FRAME",
    3: "MAV_MISSION_UNSUPPORTED",
    4: "MAV_MISSION_NO_SPACE",
    5: "MAV_MISSION_INVALID",
    6: "MAV_MISSION_INVALID_PARAM1",
    7: "MAV_MISSION_INVALID_PARAM2",
    8: "MAV_MISSION_INVALID_PARAM3",
    9: "MAV_MISSION_INVALID_PARAM4",
    10: "MAV_MISSION_INVALID_PARAM5_X",
    11: "MAV_MISSION_INVALID_PARAM6_Y",
    12: "MAV_MISSION_INVALID_PARAM7_Z",
    13: "MAV_MISSION_INVALID_SEQUENCE",
    14: "MAV_MISSION_DENIED",
    15: "MAV_MISSION_OPERATION_CANCELLED"
}

MAV_LANDED_STATE = {
    0: "MAV_LANDED_STATE_UNDEFINED",
    1: "MAV_LANDED_STATE_ON_GROUND",
    2: "MAV_LANDED_STATE_IN_AIR",
    3: "MAV_LANDED_STATE_TAKEOFF",
    4: "MAV_LANDED_STATE_LANDING"
}

def get_mav_result_string(result_code: int | None) -> str:
    if result_code is None:
        return "UNKNOWN"

    return MAV_RESULTS.get(result_code, f"UNKNOWN CODE: {result_code}")

def get_mav_mission_result_string(result_code: int | None) -> str:
    if result_code is None:
        return "UNKNOWN"

    return MAV_MISSION_RESULT.get(result_code, f"UNKNOWN CODE: {result_code}")

def get_mav_landed_state_string(state_code: int | None) -> str:
    if state_code is None:
        return "UNKNOWN"

    return MAV_LANDED_STATE.get(state_code, f"UNKNOWN CODE: {state_code}")