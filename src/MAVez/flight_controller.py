# flight_controller.py
# version: 2.0.0
# Original Author: Theodore Tasman
# Creation Date: 2025-01-30
# Last Modified: 2025-09-15
# Organization: PSU UAS

"""
This module is responsible for managing the flight of ardupilot.
"""

# SITL Start Command:
# python3 ./MAVLink/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane --console --map --custom-location 38.31527628,-76.54908330,40,282.5

from logging import Logger
from MAVez.mission import Mission
from MAVez.controller import Controller
import time


class FlightController(Controller):
    """
    Manages the flight plan for ardupilot. Extends the Controller class to provide complex flight functionalities.

    Args:
        connection_string (str): The connection string to ardupilot.
        logger (Logger | None): Optional logger for logging flight events.
        craft_type (str): The type of craft ("plane" or "copter").

    Raises:
        ConnectionError: If the connection to ardupilot fails.

    Returns:
        Flight_Controller: An instance of the Flight_Controller class.
    """

    TIMEOUT_ERROR = 101  # Timeout error code
    BAD_RESPONSE_ERROR = 102  # Bad response error code
    UNKNOWN_MODE_ERROR = 111  # Unknown mode error code
    INVALID_MISSION_ERROR = 301  # Invalid mission error code

    from typing import Literal

    def __init__(self, connection_string: str="tcp:127.0.0.1:5762", logger: Logger|None=None, craft_type: Literal["plane", "copter"]="plane"):
        # Initialize the controller
        super().__init__(connection_string, logger=logger)

        self.geofence = Mission(self, type=1)  # type 1 is geofence

        # initialize mission queue
        self.mission_queue = []

        self.craft_type = craft_type

    def decode_error(self, error_code: int) -> str:
        """
        Decode an error code.

        Args:
            error_code (int): The error code to decode.

        Returns:
            str: A string describing the error.
        """

        errors_dict = {
            101: "\nTIMEOUT ERROR (101)\n",
            102: "\nBAD RESPONSE ERROR (102)\n",
            111: "\nUNKNOWN MODE ERROR (111)\n",
            301: "\nINVALID MISSION ERROR (301)\n",
        }

        return errors_dict.get(error_code, f"UNKNOWN ERROR ({error_code})")

    def takeoff(self, takeoff_mission_filename: str) -> int:
        """
        Takeoff ardupilot.

        Args:
            takeoff_mission_filename (str): The file containing the takeoff mission.

        Returns:
            int: 0 if the takeoff was successful, otherwise an error code.
        """

        # Load the takeoff mission from the file
        takeoff_mission = Mission(self)
        response = takeoff_mission.load_mission_from_file(takeoff_mission_filename)

        # verify that the mission was loaded successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Takeoff failed, mission not loaded")
            return response

        # verify that the mission is a takeoff mission
        if not takeoff_mission.is_takeoff:
            if self.logger:
                self.logger.critical("[Flight] Takeoff failed, not a takeoff mission")
            return self.INVALID_MISSION_ERROR
        

        # send the takeoff mission
        response = takeoff_mission.send_mission()

        # verify that the mission was sent successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Takeoff failed, mission not sent")
            return response

        # wait for mission to be fully received
        # Countdown from 5
        if self.logger:
            self.logger.info("[Flight] Takeoff in 5 seconds")
        for i in range(5, 0, -1):
            time.sleep(1)

        # set the mode to AUTO
        response = self.set_mode("AUTO")

        # verify that the mode was set successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Takeoff failed, mode not set to AUTO")
            return response

        # arm ardupilot
        response = self.arm()

        # verify that ardupilot was armed successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Takeoff failed, drone not armed")
            return response

        return 0

    def append_mission(self, filename) -> int:
        """
        Append a mission to the mission list.

        Args:
            filename (str): The file containing the mission to append.

        Returns:
            int: 0 if the mission was appended successfully, otherwise an error code.
        """
        # Load the mission from the file
        mission = Mission(self)
        result = mission.load_mission_from_file(filename)

        if result:
            if self.logger:
                self.logger.critical("[Flight] Could not append mission.")
            return result

        if self.logger:
            self.logger.info(
                f"[Flight] Appended mission from {filename} to mission list"
            )
        self.mission_queue.append(mission)
        return 0

    def await_waypoint_reached(self, target) -> int:
        """
        Wait for ardupilot to reach the current waypoint.

        Args:
            target (int): The target waypoint index to wait for.

        Returns:
            int: 0 if the waypoint was reached successfully, otherwise an error code.
        """
        latest_waypoint = -1

        if self.logger:
            self.logger.info(f"[Flight] Waiting for waypoint {target} to be reached")

        while latest_waypoint < target:
            response = self.receive_mission_item_reached()

            if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
                return response

            latest_waypoint = response

        if self.logger:
            self.logger.info(f"[Flight] Waypoint {target} reached")
        return 0

    def wait_and_send_next_mission(self) -> int:
        """
        Waits for the last waypoint to be reached, clears the mission, sends the next mission, sets mode to auto.

        Returns:
            int: 0 if the next mission was sent successfully, otherwise an error code.
        """
        # Get the current mission
        current_mission = self.mission_queue.pop(0)

        # if the mission list is empty, return
        if len(self.mission_queue) == 0:
            if self.logger:
                self.logger.info("[Flight] No more missions in list")
            return 0

        # otherwise, set the next mission to the next mission in the list
        else:
            if self.logger:
                self.logger.info(
                    f"[Flight] Queuing next mission in list of {len(self.mission_queue)} missions"
                )
            next_mission = self.mission_queue[0]

        # calculate the target index
        target_index = len(current_mission) - 1

        # Wait for the target index to be reached
        response = self.await_waypoint_reached(target_index)

        # verify that the response was received
        if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
            if self.logger:
                self.logger.critical("[Flight] Failed to wait for next mission.")
            return response

        # Clear the mission
        response = current_mission.clear_mission()
        if response:
            if self.logger:
                self.logger.critical("[Flight] Failed to send next mission.")
            return response

        # Send the next mission
        result = next_mission.send_mission()
        if result:
            if self.logger:
                self.logger.critical("[Flight] Failed to send next mission.")
            return result

        # set the mode to AUTO
        response = self.set_mode("AUTO")

        # verify that the mode was set successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Failed to send next mission.")
            return response

        if self.logger:
            self.logger.info("[Flight] Next mission sent")
        return result

    def await_landing(self, timeout=60) -> int:
        """
        Wait for ardupilot to signal landed.

        Args:
            timeout (int): The maximum time to wait for the landing status in seconds.

        Returns:
            int: 0 if the landing was successful, otherwise an error code.
        """
        landing_status = -1

        # start receiving landing status
        response = self.set_message_interval(
            message_type=245, interval=1e6
        )  # 245 is landing status (EXTENDED_SYS_STATE), 1e6 is 1 second
        if response:
            if self.logger:
                self.logger.critical("[Flight] Failed waiting for landing.")
            return response

        # wait for landing status to be landed
        start_time = time.time()
        while (
            landing_status != 1
        ):  # 1 for landed, 2 for in air, 3 for taking off, 4 for currently landing, 0 for unknown
            # check for timeout
            if time.time() - start_time > timeout:
                response = self.TIMEOUT_ERROR
                if self.logger:
                    self.logger.error("[Flight] Timed out waiting for landing.")
                return response

            # get the landing status
            response = self.receive_landing_status()

            # verify that the response was received
            if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
                if self.logger:
                    self.logger.error("[Flight] Failed waiting for landing.")
                return response

            landing_status = response

        # stop receiving landing status
        response = self.disable_message_interval(
            message_type=245
        )  # 245 is landing status (EXTENDED_SYS_STATE)
        if response:
            if self.logger:
                self.logger.error("[Flight] Error waiting for landing.")
            return response

        return 0

    def jump_to_next_mission_item(self) -> int:
        """
        Jump to the next mission item.

        Returns:
            int: 0 if the jump was successful, otherwise an error code.
        """

        if self.logger:
            self.logger.info("[Flight] Waiting for current mission index")
        # wait for the current mission target to be received (should be broadcast by default)
        response = self.receive_current_mission_index()
        if response == self.TIMEOUT_ERROR:
            return response

        # jump to the next mission item
        response = self.set_current_mission_index(response + 1)
        if response:
            return response

        return 0

    def await_channel_input(
        self, channel, value, wait_time=120, value_tolerance=100
    ) -> int:
        """
        Wait for a specified rc channel to reach a given value

        Args:
            channel (int): The channel number to wait for.
            value (int): The value to wait for.
            wait_time (int): The maximum time to wait for the channel to be set in seconds.
            value_tolerance (int): The tolerance range for the set value.

        Returns:
            int: 0 if the channel was set to the desired value, otherwise an error code
        """
        latest_value = -float("inf")
        start_time = time.time()

        # set the channel to be received
        channel = f"chan{channel}_raw"

        if self.logger:
            self.logger.info(
                f"[Flight] Waiting for channel {channel} to be set to {value}"
            )
        # only wait for the channel to be set for a certain amount of time
        while time.time() - start_time < wait_time:
            # get channel inputs
            response = self.receive_channel_input()

            # verify that the response was received
            if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
                if self.logger:
                    self.logger.critical("[Flight] Failed waiting for channel input.")
                return response

            # channel key is 'chanX_raw' where X is the channel number
            latest_value = getattr(response, channel)

            # print(f'Latest value: {latest_value}')
            # check if the value is within the tolerance range
            if (
                latest_value > value - value_tolerance
                and latest_value < value + value_tolerance
            ):
                if self.logger:
                    self.logger.info(
                        f"[Flight] Channel {channel} set to {latest_value}"
                    )
                return 0

        if self.logger:
            self.logger.critical(
                f"[Flight] Timed out waiting for channel {channel} to be set to {value}"
            )
        return self.TIMEOUT_ERROR
