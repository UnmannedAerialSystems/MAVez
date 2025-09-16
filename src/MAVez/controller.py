# mav_controller.py
# version: 2.0.0
# Author: Theodore Tasman
# Creation Date: 2025-01-30
# Last Modified: 2025-09-15
# Organization: PSU UAS

"""
This module is responsible for controlling ardupilot.
"""

from logging import Logger
from pymavlink import mavutil
from pyparsing import Any # type: ignore[import]
from MAVez.coordinate import Coordinate
from MAVez.safe_logger import SafeLogger


class Controller:
    """
    Controller class for atomic MAVLink communication with ardupilot.

    Args:
        connection_string (str): The connection string for ardupilot. Default is "tcp:127.0.0.1:5762" used for SITL.
        baud (int): The baud rate for the connection. Default is 57600.
        logger: Logger instance for logging messages (optional).

    Raises:
        ConnectionError: If the connection to ardupilot fails.

    Returns:
        A Controller instance that can be used to communicate with ardupilot.
    """

    # error codes
    TIMEOUT_ERROR = 101
    BAD_RESPONSE_ERROR = 102
    UNKNOWN_MODE = 111

    TIMEOUT_DURATION = 5  # timeout duration in seconds

    def __init__(self, connection_string: str = "tcp:127.0.0.1:5762", baud: int = 57600, logger: Logger | None = None):
        """
        Initialize the controller.

        Args:
            connection_string (str): The connection string for ardupilot.
            baud (int): The baud rate for the connection.
            logger: Logger instance for logging messages (optional).

        Raises:
            ConnectionError: If the connection to ardupilot fails.

        Returns:
            None
        """
        self.logger = SafeLogger(logger)

        self.master = mavutil.mavlink_connection(connection_string, baud=baud)  # type: ignore

        response = self.master.wait_heartbeat(  # type: ignore
            blocking=True, timeout=self.TIMEOUT_DURATION
        ) 

        # check if the connection was successful
        if not response:
            self.logger.error("[Controller] Connection failed")
            raise ConnectionError("Connection failed")

    def decode_error(self, error_code: int) -> str:
        """
        Decode the error code into a human-readable string.

        Args:
            error_code (int): The error code to decode.

        Returns:
            str: A human-readable error message.
        """
        errors_dict = {
            101: "\nRESPONSE TIMEOUT ERROR (101)\n",
            102: "\nBAD RESPONSE ERROR (102)\n",
            111: "\nUNKNOWN MODE ERROR (111)\n",
        }

        return errors_dict.get(error_code, f"UNKNOWN ERROR ({error_code})")

    def receive_mission_request(self) -> int:
        """
        Wait for a mission request from ardupilot.

        Returns:
            int: Mission index if a mission request was received, 101 if the response timed out, 102 if a bad response was received.
        """

        # MISSION_REQUEST IS DEPRECATED, but likely still used. (replaced by MISSION_REQUEST_INT)
        # Always respond with MISSION_ITEM_INT.
        # If this stops working, try receiving MISSION_REQUEST_INT instead.
        response = self.master.recv_match(  # type: ignore
            type="MISSION_REQUEST", blocking=True
        )
        if response:
            self.logger.info(
                    f"[Controller] Received mission request: {response.seq}" # type: ignore
                )
            return response.seq if response.seq is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Mission request timed out")
            return self.TIMEOUT_ERROR

    def receive_mission_ack(self) -> int:
        """
        Wait for a mission ack from ardupilot.

        Returns:
            int: 0 if the mission ack was received, error code if there was an error, 101 if the response timed out.
        """

        response = self.master.recv_match(
            type="MISSION_ACK", blocking=True
        )
        if response:
            if response.type == 0:
                if self.logger:
                    self.logger.info("[Controller] Received mission ack.")
                return 0
            else:
                self.logger.error(
                    f"[Controller] Mission ack error: {response.type}"
                )
                return response.type if response.type is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Mission ack timed out")
            return self.TIMEOUT_ERROR

    def send_message(self, message):
        """
        Send a MAVLink message to ardupilot.

        Args:
            message: The MAVLink message to send.

        Returns:
            None
        """
        self.master.mav.send(message) # type: ignore

    def send_mission_count(self, count, mission_type=0) -> int:
        """
        Send the mission count to ardupilot.

        Args:
            count (int): The number of mission items.
            mission_type (int): The type of mission (default is 0 for MISSION_TYPE 0).

        Returns:
            int: 0 if the mission count was sent successfully.
        """

        self.master.mav.mission_count_send( # type: ignore
            0,  # target_system
            0,  # target_component
            count,  # count
            mission_type,  # mission_type
        )
        self.logger.info(f"[Controller] Sent mission count: {count}")
        return 0

    def receive_mission_item_reached(self) -> int:
        """
        Wait for a mission item reached message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            int: The sequence number of the reached mission item if received, TIMEOUT_ERROR (101) if the response timed out.
        """

        response = self.master.recv_match(
            type="MISSION_ITEM_REACHED", blocking=True
        )
        if response:
            self.logger.info(
                f"[Controller] Received mission item reached: {response.seq}"
            )
            return response.seq if response.seq is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Mission item reached timed out")
            return self.TIMEOUT_ERROR

    def send_clear_mission(self) -> int:
        """
        Clear the mission on ardupilot.

        Returns:
            int: 0 if the mission was cleared successfully
        """

        self.master.waypoint_clear_all_send() # type: ignore
        self.logger.info("[Controller] Sent clear mission")
        return 0

    def set_mode(self, mode) -> int:
        """
        Set the ardupilot mode.

        Args:
            mode (str): The mode to set ardupilot to. Options include: "AUTO", "GUIDED", "FBWA", etc...

        Returns:
            int: 0 if the mode was set successfully, 111 if the mode is unknown, 101 if the response timed out.
        """

        if mode not in self.master.mode_mapping(): # type: ignore
            self.logger.error(f"[Controller] Unknown mode: {mode}")
            return self.UNKNOWN_MODE

        mode_id = self.master.mode_mapping()[mode] # type: ignore
        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # command
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # param1
            mode_id,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        self.master.mav.send(message) # type: ignore

        response = self.master.recv_match(
            type="COMMAND_ACK", blocking=True
        )
        if response:
            if response.result == 0:
                self.logger.info(f"[Controller] Set mode to {mode}")
                return 0
            else:
                self.logger.error(
                    f"[Controller] Failed to set mode: {response.result}"
                )
                return response.result if response.result is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Set mode timed out")
            return self.TIMEOUT_ERROR

    def arm(self, force=False) -> int:
        """
        Arm ardupilot

        Args:
            force (bool): If True, ardupilot will be armed regardless of its state.

        Returns:
            int: 0 if ardupilot was armed successfully, error code if there was an error, 101 if the response timed out.
        """

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
            0,  # confirmation
            1,  # param1
            21196 if force else 0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        self.master.mav.send(message) # type: ignore

        response = self.master.recv_match(
            type="COMMAND_ACK", blocking=True
        )
        if response:
            if response.result == 0:
                self.logger.info("[Controller] Drone armed successfully")
                return 0
            else:
                self.logger.error(
                        f"[Controller] Failed to arm drone: {response.result}"
                    )
                return response.result if response.result is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Arm command timed out")
            return self.TIMEOUT_ERROR

    def disarm(self, force=False) -> int:
        """
        Disarm ardupilot.

        Args:
            force (bool): If True, ardupilot will be disarmed regardless of its state.

        Returns:
            int: 0 if ardupilot was disarmed successfully, error code if there was an error, 101 if the response timed out.
        """

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
            0,  # confirmation
            0,  # param1
            21196 if force else 0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        self.master.mav.send(message) # type: ignore

        response = self.master.recv_match(
            type="COMMAND_ACK", blocking=True
        )
        if response:
            if response.result == 0:
                self.logger.info("[Controller] Drone disarmed successfully")
                return 0
            else:
                self.logger.error(
                    f"[Controller] Failed to disarm drone: {response.result}"
                    )
                return response.result if response.result is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Disarm command timed out")
            return self.TIMEOUT_ERROR

    def enable_geofence(self) -> int:
        """
        Enable the geofence.

        Returns:
            int: 0 if the geofence was enabled successfully, 101 if the response timed out.
        """

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,  # command
            0,  # confirmation
            1,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        self.master.mav.send(message) # type: ignore

        response = self.master.recv_match(
            type="COMMAND_ACK", blocking=True
        )
        if response:
            if response.result == 0:
                self.logger.info("[Controller] Geofence enabled successfully")
                return 0
            else:
                self.logger.error(
                        f"[Controller] Failed to enable geofence: {response.result}"
                    )
                return response.result if response.result is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Geofence enable command timed out")
            return self.TIMEOUT_ERROR

    def disable_geofence(self, floor_only=False) -> int:
        """
        Disable the geofence.

        Args:
            floor_only (bool): If True, only the floor of the geofence will be disabled.

        Returns:
            int: 0 if the geofence was disabled successfully, 101 if the response timed out.
        """

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE,  # command
            0,  # confirmation
            2 if floor_only else 0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        self.master.mav.send(message) # type: ignore

        response = self.master.recv_match(
            type="COMMAND_ACK", blocking=True
        )
        if response:
            if response.result == 0:
                if self.logger:
                    self.logger.info("[Controller] Geofence disabled successfully")
                return 0
            else:
                self.logger.error(
                    f"[Controller] Failed to disable geofence: {response.result}"
                )
                return response.result if response.result is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Geofence disable command timed out")
            return self.TIMEOUT_ERROR

    def set_home(self, home_coordinate=Coordinate(0, 0, 0)) -> int:
        """
        Set the home location.

        Args:
            home_coordinate (Coordinate): The home coordinate to set. If the coordinate is (0, 0, 0), the current GPS location will be used.

        Returns:
            int: 0 if the home location was set successfully, error code if there was an error, 101 if the response timed out.
        """

        # use_current is set to True if the home coordinate is (0, 0, 0)
        use_current = home_coordinate == (0, 0, 0)
        # if alt is 0, use the current altitude

        if home_coordinate.alt == 0:
            current_pos = self.receive_gps()
            home_coordinate.alt = current_pos.alt if isinstance(current_pos, Coordinate) else 0
        else:
            home_coordinate.alt = home_coordinate.alt


        message = self.master.mav.command_int_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            0,  # frame - MAV_FRAME_GLOBAL
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # command
            0,  # current
            0,  # auto continue
            1 if use_current else 0,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            home_coordinate.lat,  # param5
            home_coordinate.lon,  # param6
            int(home_coordinate.alt),  # param7
        )

        self.master.mav.send(message) # type: ignore

        response = self.master.recv_match(
            type="COMMAND_ACK", blocking=True
        )
        if response:
            if response.result == 0:
                self.logger.info(
                    f"[Controller] Home location set to {home_coordinate}"
                )
                return 0
            else:
                self.logger.error(
                    f"[Controller] Failed to set home location: {response.result}"
                    )
                return response.result if response.result is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Set home location command timed out")
            return self.TIMEOUT_ERROR

    def set_servo(self, servo_number, pwm) -> int:
        """
        Set the a servo to a specified PWM value.

        Args:
            servo_number (int): The servo number to set.
            pwm (int): The PWM value to set the servo to.

        Returns:
            int: 0 if the servo was set successfully, error code if there was an error, 101 if the response timed out.
        """
        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
            0,  # confirmation
            servo_number,  # param1
            pwm,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        self.master.mav.send(message) # type: ignore
        response = self.master.recv_match(
            type="COMMAND_ACK", blocking=True
        )
        if response:
            if response.result == 0:
                self.logger.info(f"[Controller] Set servo {servo_number} to {pwm}")
                return 0
            else:
                self.logger.error(
                    f"[Controller] Failed to set servo {servo_number}: {response.result}"
                )
                return response.result if response.result is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Set servo command timed out")
            return self.TIMEOUT_ERROR
    
    def receive_channel_input(self) -> int | Any:
        """
        Wait for an RC_CHANNELS message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            response if an RC_CHANNELS message was received, 101 if the response timed out
        """

        response = self.master.recv_match(
            type="RC_CHANNELS", blocking=True
        )
        if response:
            self.logger.info(
                f"[Controller] Received channel input from {response.chancount} channels"
            )
            return response
        else:
            self.logger.error("[Controller] Receive channel input timed out")
            return self.TIMEOUT_ERROR

    def receive_wind(self) -> int | Any:
        """
        Wait for a wind_cov message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            response if a wind_cov message was received, 101 if the response timed out
        """
        response = self.master.recv_match(
            type="WIND_COV", blocking=True
        )
        if response:
            self.logger.info("[Controller] Received wind data")
            return response
        else:
            self.logger.error("[Controller] Receive wind data timed out")
            return self.TIMEOUT_ERROR

    def receive_gps(self) -> int | Coordinate:
        """
        Wait for a gps_raw_int message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            Coordinate: A Coordinate object containing the GPS data if received, TIMEOUT_ERROR (101) if the response timed out.
        """
        response = self.master.recv_match(
            type="GLOBAL_POSITION_INT", blocking=True
        )

        if response:
            if response.lat is None or response.lon is None or response.alt is None:
                self.logger.error("[Controller] Received invalid GPS data")
                return self.BAD_RESPONSE_ERROR
            self.logger.info(
                f"[Controller] Received GPS data, lat: {response.lat}, lon: {response.lon}, alt: {response.alt / 1000}, heading: {response.hdg}"
            )
            return Coordinate(
                response.lat,
                response.lon,
                response.alt / 1000,
                use_int=False,
                heading=response.hdg,
            )  # convert to meters, lat and lon are in degrees e7

        else:
            self.logger.error("[Controller] Receive GPS data timed out")
            return self.TIMEOUT_ERROR

    def receive_landing_status(self, timeout=TIMEOUT_DURATION) -> int:
        """
        Wait for a landed_state message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            int: The landing state if received, TIMEOUT_ERROR (101) if the response timed out, 0 if the state is undefined, 1 if on ground, 2 if in air, 3 if taking off, 4 if landing.
        """
        response = self.master.recv_match(
            type="EXTENDED_SYS_STATE", blocking=True
        )
        if response:
            self.logger.info(
                f"[Controller] Received landing status: {response.landed_state}"
            )
            return response.landed_state if response.landed_state is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Receive landing status timed out")
            return self.TIMEOUT_ERROR

    def set_message_interval(self, message_type, interval) -> int:
        """
        Set the message interval for the specified message type.

        Args:
            message_type (str): The type of message to set the interval for.
            interval (int): The interval in milliseconds to set for the message type.
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            int: 0 if the message interval was set successfully, error code if there was an error, 101 if the response timed out.
        """

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # command
            0,  # confirmation
            message_type,  # param1
            interval,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        self.master.mav.send(message) # type: ignore

        response = self.master.recv_match(
            type="COMMAND_ACK", blocking=True
        )
        if response:
            if response.result == 0:
                self.logger.info(
                    f"[Controller] Set message interval for {message_type} to {interval}"
                )
                return 0
            else:
                self.logger.error(
                    f"[Controller] Failed to set message interval: {response.result}"
                )
                return response.result if response.result is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Set message interval command timed out")
            return self.TIMEOUT_ERROR

    def disable_message_interval(self, message_type) -> int:
        """
        Disable the message interval for the specified message type.

        Args:
            message_type (str): The type of message to disable the interval for.
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            int: 0 if the message interval was disabled successfully, error code if there was an error, 101 if the response timed out.
        """

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # command
            0,  # confirmation
            message_type,  # param1
            -1,  # param2 # -1 disables the message
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        self.master.mav.send(message) # type: ignore

        response = self.master.recv_match(
            type="COMMAND_ACK", blocking=True
        )
        if response:
            if response.result == 0:
                self.logger.info(
                    f"[Controller] Disabled message interval for {message_type}"
                )
                return 0
            else:
                self.logger.error(
                    f"[Controller] Failed to disable message interval: {response.result}"
                )
                return response.result if response.result is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error(
                "[Controller] Disable message interval command timed out"
            )
            return self.TIMEOUT_ERROR

    def receive_current_mission_index(self) -> int:
        """
        Get the current mission index.

        Returns:
            int: The current mission index if received, TIMEOUT_ERROR (101) if the response timed out.
        """

        response = self.master.recv_match(
            type="MISSION_CURRENT", blocking=True
        )
        if response:
            self.logger.info(
                f"[Controller] Received current mission index: {response.seq}"
            )
            return response.seq if response.seq is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error(
                "[Controller] Receive current mission index timed out"
            )
            return self.TIMEOUT_ERROR

    def set_current_mission_index(self, index) -> int:
        """
        sets the target mission index to the specified index

        Args:
            index (int): The index to set as the current mission index.

        Returns:
            int: 0 if the current mission index was set successfully, error code if there was an error, 101 if the response timed out.
        """

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,  # command
            0,  # confirmation
            index,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )
        self.master.mav.send(message) # type: ignore
        response = self.master.recv_match(
            type="COMMAND_ACK", blocking=True
        )
        if response:
            if response.result == 0:
                self.logger.info(
                    f"[Controller] Set current mission index to {index}"
                )
                return 0
            else:
                self.logger.error(
                    f"[Controller] Failed to set current mission index: {response.result}"
                )
                return response.result if response.result is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error(
                "[Controller] Set current mission index command timed out"
            )
            return self.TIMEOUT_ERROR

    def start_mission(self, start_index, end_index) -> int:
        """
        Start the mission at the specified index.

        Args:
            start_index (int): The index to start the mission from.
            end_index (int): The index to end the mission at.

        Returns:
            int: 0 if the mission was started successfully, error code if there was an error, 101 if the response timed out.
        """

        message = self.master.mav.command_long_encode( # type: ignore
            0,  # target_system
            0,  # target_component
            mavutil.mavlink.MAV_CMD_MISSION_START,  # command
            0,  # confirmation
            start_index,  # param1
            end_index,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

        self.master.mav.send(message) # type: ignore

        response = self.master.recv_match(
            type="COMMAND_ACK", blocking=True
        )
        if response:
            if response.result == 0:
                self.logger.info(
                    f"[Controller] Started mission from {start_index} to {end_index}"
                )
                return 0
            else:
                self.logger.error(
                    f"[Controller] Failed to start mission: {response.result}"
                )
                return response.result if response.result is not None else self.BAD_RESPONSE_ERROR
        else:
            self.logger.error("[Controller] Start mission command timed out")
            return self.TIMEOUT_ERROR

    def receive_attitude(self, timeout=TIMEOUT_DURATION) -> int | Any:
        """
        Wait for an attitude message from ardupilot.

        Args:
            timeout (int): The timeout duration in seconds. Default is 5 seconds.

        Returns:
            response if an attitude message was received, TIMEOUT_ERROR (101) if the response timed out.
        """

        response = self.master.recv_match(
            type="ATTITUDE", blocking=True
        )
        if response:
            self.logger.info(
                 f"[Controller] Received attitude data: roll={response.roll}, pitch={response.pitch}, yaw={response.yaw}"
            )
            return response
        else:
            self.logger.error("[Controller] Receive attitude data timed out")
            return self.TIMEOUT_ERROR
