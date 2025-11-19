# flight_controller.py
# version: 3.1.0
# Original Author: Theodore Tasman
# Creation Date: 2025-01-30
# Last Modified: 2025-11-14
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
        baud (int): The baud rate for the connection. Default is 57600.
        logger (Logger | None): Optional logger for logging flight events.
        message_host (str): The host for messaging.
        message_host (str): The host for the messaging system. Default is "127.0.0.1".
        message_port (int): The port for the messaging system. Default is 5555.
        message_topic (str): The topic prefix for the messaging system. Default is "".
        timesync (bool): Whether to enable time synchronization. Default is False.

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

    def __init__(self, connection_string: str="tcp:127.0.0.1:5762", 
                 baud: int=57600, 
                 logger: Logger|None=None, 
                 message_host: str="127.0.0.1", 
                 message_port: int=5555, 
                 message_topic: str="",
                 timesync: bool=False) -> None:
        # Initialize the controller
        super().__init__(connection_string, logger=logger, baud=baud, message_host=message_host, message_port=message_port, message_topic=message_topic, timesync=timesync)

        self.geofence = Mission(self, type=1)  # type 1 is geofence

        # initialize mission queue
        self.mission_queue = []

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

    async def takeoff(self, takeoff_mission_filename: str) -> int:
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

        if response:
            self.logger.critical("[Flight] Takeoff failed, could not load mission")
            return response

        if not takeoff_mission.is_takeoff:
            self.logger.critical("[Flight] Takeoff failed, mission has no takeoff")
            return self.INVALID_MISSION_ERROR

        self.mission_queue.append(takeoff_mission)

        # send the takeoff mission
        response = await takeoff_mission.send_mission()

        # verify that the mission was sent successfully
        if response:
            self.logger.critical("[Flight] Takeoff failed, mission not sent")
            return response

        # set the mode to AUTO
        response = await self.set_mode("AUTO")

        # verify that the mode was set successfully
        if response:
            self.logger.critical("[Flight] Takeoff failed, mode not set to AUTO")
            return response
        
        # reset mission index to 0
        response = await self.set_current_mission_index(0, reset=True)
        if response:
            self.logger.critical("[Flight] Takeoff failed, could not reset mission index")
            return response

        # arm ardupilot
        response = await self.arm()

        # verify that ardupilot was armed successfully
        if response:
            self.logger.critical("[Flight] Takeoff failed, vehicle not armed")
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
            self.logger.critical("[Flight] Could not append mission.")
            return result

        self.logger.info(f"[Flight] Appended mission from {filename} to mission list")
        self.mission_queue.append(mission)
        return 0

    async def wait_for_waypoint(self, target) -> int:
        """
        Wait for ardupilot to reach the current waypoint.

        Args:
            target (int): The target waypoint index to wait for.

        Returns:
            int: 0 if the waypoint was reached successfully, otherwise an error code.
        """
        latest_waypoint = -1

        self.logger.debug(f"[Flight] Waiting for waypoint {target} to be reached")

        while latest_waypoint < target:
            response = await self.receive_mission_item_reached()

            if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
                return response

            latest_waypoint = response

        self.logger.info(f"[Flight] Waypoint {target} reached")
        return 0

    async def auto_send_next_mission(self) -> int:
        """
        Waits for the last waypoint to be reached, clears the mission, sends the next mission, sets mode to auto.

        Returns:
            int: 0 if the next mission was sent successfully, otherwise an error code.
        """
        # Get the current mission
        current_mission = self.mission_queue.pop(0)

        # if the mission list is empty, return
        if len(self.mission_queue) == 0:
            self.logger.info("[Flight] No more missions in list")
            return 0

        # otherwise, set the next mission to the next mission in the list
        else:
            self.logger.info(f"[Flight] Queuing next mission in list of {len(self.mission_queue)} missions")
            next_mission = self.mission_queue[0]

        # calculate the target index
        target_index = len(current_mission) - 1

        # Wait for the target index to be reached
        response = await self.wait_for_waypoint(target_index)

        # verify that the response was received
        if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
            self.logger.critical("[Flight] Failed to wait for next mission.")
            return response

        # Clear the mission
        response = await current_mission.clear_mission()
        if response:
            self.logger.critical("[Flight] Failed to send next mission.")
            return response

        # Send the next mission
        result = await next_mission.send_mission()
        if result:
            self.logger.critical("[Flight] Failed to send next mission.")
            return result

        # set the mode to AUTO
        response = await self.set_mode("AUTO")

        # verify that the mode was set successfully
        if response:
            self.logger.critical("[Flight] Failed to send next mission.")
            return response

        self.logger.info("[Flight] Next mission sent")
        return result

    async def wait_for_landing(self, timeout=60) -> int:
        """
        Wait for ardupilot to signal landed.

        Args:
            timeout (int): The maximum time to wait for the landing status in seconds.

        Returns:
            int: 0 if the landing was successful, otherwise an error code.
        """
        landing_status = -1

        # start receiving landing status
        response = await self.set_message_interval(
            message_type=245, interval=100000
        )  # 245 is landing status (EXTENDED_SYS_STATE), 1e6 is 1 second
        if response:
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
                self.logger.error("[Flight] Timed out waiting for landing.")
                return response

            # get the landing status
            response = await self.receive_landing_status()

            # verify that the response was received
            if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
                self.logger.error("[Flight] Failed waiting for landing.")
                return response

            landing_status = response

        # stop receiving landing status
        response = await self.disable_message_interval(
            message_type=245
        )  # 245 is landing status (EXTENDED_SYS_STATE)
        if response:
            self.logger.error("[Flight] Error waiting for landing.")
            return response

        return 0

    async def jump_to_next_mission_item(self) -> int:
        """
        Jump to the next mission item.

        Returns:
            int: 0 if the jump was successful, otherwise an error code.
        """

        self.logger.debug("[Flight] Waiting for current mission index")
        # wait for the current mission target to be received (should be broadcast by default)
        response = await self.receive_current_mission_index()
        if response == self.TIMEOUT_ERROR:
            return response

        # jump to the next mission item
        response = await self.set_current_mission_index(response + 1)
        if response:
            return response

        return 0

    async def wait_for_channel_input(self, channel, value, wait_time=120, value_tolerance=100) -> int:
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

        self.logger.debug(f"[Flight] Waiting for channel {channel} to be set to {value}")

        # only wait for the channel to be set for a certain amount of time
        while time.time() - start_time < wait_time:
            # get channel inputs
            response = await self.receive_channel_input()

            # verify that the response was received
            if response == self.TIMEOUT_ERROR or response == self.BAD_RESPONSE_ERROR:
                self.logger.critical("[Flight] Failed waiting for channel input.")
                return response

            # channel key is 'chanX_raw' where X is the channel number
            latest_value = getattr(response, channel)

            # check if the value is within the tolerance range
            if (
                latest_value > value - value_tolerance
                and latest_value < value + value_tolerance
            ):
                self.logger.info(f"[Flight] Channel {channel} set to {latest_value}")

                return 0

        self.logger.critical(
            f"[Flight] Timed out waiting for channel {channel} to be set to {value}"
        )
        return self.TIMEOUT_ERROR
    
    async def set_geofence(self, geofence_filename: str) -> int:
        """
        Send and enable the geofence from a file.

        Args:
            geofence_filename (str): The file containing the geofence mission.

        Returns:
            int: 0 if the geofence was set successfully, otherwise an error code.
        """
        # Load the geofence mission from the file
        response = self.geofence.load_mission_from_file(geofence_filename)

        if response:
            self.logger.critical("[Flight] Geofence failed, could not load mission")
            return response

        if not self.geofence.is_geofence:
            self.logger.critical("[Flight] Geofence failed, mission is not a geofence")
            return self.INVALID_MISSION_ERROR

        # send the geofence mission
        response = await self.geofence.send_mission()

        # verify that the mission was sent successfully
        if response:
            self.logger.critical("[Flight] Geofence failed, mission not sent")
            return response

        self.logger.debug("[Flight] Geofence sent")

        response = await self.enable_geofence()
        if response:
            self.logger.critical("[Flight] Geofence failed, could not be enabled")
            return response
        
        self.logger.debug("[Flight] Geofence set and enabled")
        return 0
    
    