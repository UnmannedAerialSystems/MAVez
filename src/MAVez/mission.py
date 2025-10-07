# mission.py
# version: 3.0.0
# Author: Theodore Tasman
# Creation Date: 2025-01-30
# Last Modified: 2025-09-24
# Organization: PSU UAS

"""
An ardupilot mission.
"""

from MAVez.mission_item import MissionItem
from MAVez.coordinate import Coordinate
from MAVez.controller import Controller
import time


class Mission:
    """
    Represents a mission for ardupilot.

    Args:
        controller (Controller): The controller instance to send the mission through.
        type (int): The type of the mission, default is 0 (waypoint mission).

    Returns:
        Mission: An instance of the Mission class.
    """

    # class wide error codes
    # if an error is function specific, it will be defined in the function and documented in class docstring
    TIMEOUT_ERROR = 101

    # time to wait for mission to be sent
    MISSION_SEND_TIMEOUT = 20  # seconds

    def __init__(self, controller: Controller, type: int=0):
        self.controller = controller
        self.type = type
        self.mission_items: list[MissionItem] = []
        self.is_takeoff = False
        self.is_landing = False
        self.is_geofence = self.type == 1

    def __str__(self) -> str:
        output = ""
        for mission_item in self.mission_items:
            output += f"{mission_item.seq}\t{mission_item.current}\t{mission_item.frame}\t{mission_item.command}\t{mission_item.param1}\t{mission_item.param2}\t{mission_item.param3}\t{mission_item.param4}\t{mission_item.x}\t{mission_item.y}\t{mission_item.z}\t{mission_item.auto_continue}\n"

        return output

    __repr__ = __str__

    def decode_error(self, error_code: int) -> str:
        """
        Decode an error code into a human-readable string.

        Args:
            error_code (int): The error code to decode.
        Returns:
            str: A human-readable string describing the error.
        """

        error_codes = {
            201: "\nFILE NOT FOUND ERROR (201)\n",
            202: "\nEMPTY FILE ERROR (202)\n",
            203: "\nINVALID START INDEX ERROR (203)\n",
            204: "\nINVALID END INDEX ERROR (204)\n",
        }

        return error_codes.get(error_code, f"\nUNKNOWN ERROR ({error_code})\n")

    def load_mission_from_file(
        self, filename: str, start: int=0, end: int=-1, first_seq: int=-1, overwrite: bool=True
    ):
        """
        Load a QGC WPL 110 mission from a file. For details on the file format, see: https://mavlink.io/en/file_formats/

        Args:
            filename (str): The path to the file containing the mission.
            start (int): The line number to start loading from, default is 0.
            end (int): The line number to stop loading at, default is -1 (load to the end).
            first_seq (int): The sequence number to start from, default is -1 (use the sequence number from the file).
            overwrite (bool): Whether to overwrite the existing mission items, default is True.

        Returns:
            int: 0 if the mission was loaded successfully, or an error code if there was an error.
        """

        FILE_NOT_FOUND = 201
        FILE_EMPTY = 202
        START_OUT_OF_RANGE = 203
        END_OUT_OF_RANGE = 204

        try:
            with open(filename, "r") as file:
                lines = file.readlines()
        except FileNotFoundError:
            if self.controller.logger:
                self.controller.logger.error(f"[Mission] File not found: {filename}")
            return FILE_NOT_FOUND

        if len(lines) == 0:
            if self.controller.logger:
                self.controller.logger.error(f"[Mission] File is empty: {filename}")
            return FILE_EMPTY

        elif start >= len(lines) - 1:
            if self.controller.logger:
                self.controller.logger.error(
                    f"[Mission] Start index out of range: {start} >= {len(lines) - 1}"
                )
            return START_OUT_OF_RANGE

        elif end != -1 and end >= len(lines) - 1:
            if self.controller.logger:
                self.controller.logger.error(
                    f"[Mission] End index out of range: {end} >= {len(lines) - 1}"
                )
            return END_OUT_OF_RANGE

        # prevent re-loading of mission items to the same mission
        if overwrite:
            self.mission_items = []

        # slice out intended lines
        # if no end is specified, slice to the end of the file
        # index + 1 to skip the header
        if end == -1:
            lines = lines[start + 1 :]
        else:
            lines = lines[start + 1 : end + 1]

        count = 0
        for line in lines:
            # skip empty lines
            if line == "\n":
                continue
            # remove comments
            line = line.split("#")[0].strip()
            parts = line.strip().split("\t")
            # skip empty lines
            if not parts:
                continue
            # jump seq ahead if first_seq is not -1
            if first_seq != -1:
                seq = first_seq
                first_seq += 1
            else:
                seq = int(parts[0])

            current = int(parts[1])
            frame = int(parts[2])
            command = int(parts[3])

            # set flags for special mission types
            if command == 22:
                self.is_takeoff = True
            if command == 21:
                self.is_landing = True

            param1 = float(parts[4])
            param2 = float(parts[5])
            param3 = float(parts[6])
            param4 = float(parts[7])
            x = float(parts[8])
            y = float(parts[9])
            z = float(parts[10])
            auto_continue = int(parts[11])

            item_coordinate = Coordinate(x, y, z)
            mission_item = MissionItem(
                seq,
                frame,
                command,
                current,
                auto_continue,
                item_coordinate,
                self.type,
                param1,
                param2,
                param3,
                param4,
            )
            self.mission_items.append(mission_item)
            count += 1

        if self.controller.logger:
            self.controller.logger.info(
                f"[Mission] Loaded {count} mission items from {filename}"
            )
        return 0

    def save_mission_to_file(self, filename: str) -> int:
        """
        Save the mission to a file in QGC WPL 110 format. For details on the file format, see: https://mavlink.io/en/file_formats/

        Args:
            filename (str): The path to the file to save the mission to.

        Returns:
            int: 0 if the mission was saved successfully.
        """
        with open(filename, "w") as file:
            file.write("QGC WPL 110\n")
            for mission_item in self.mission_items:
                file.write(
                    f"{mission_item.seq}\t{mission_item.current}\t{mission_item.frame}\t{mission_item.command}\t{mission_item.param1}\t{mission_item.param2}\t{mission_item.param3}\t{mission_item.param4}\t{mission_item.x}\t{mission_item.y}\t{mission_item.z}\t{mission_item.auto_continue}\n"
                )

        if self.controller.logger:
            self.controller.logger.info(
                f"[Mission] Saved {len(self.mission_items)} mission items to {filename}"
            )
        return 0

    def add_mission_item(self, mission_item: MissionItem) -> int:
        """
        Add a mission item to the mission.

        Args:
            mission_item (Mission_Item): The mission item to add.

        Returns:
            int: 0 if the mission item was added successfully.
        """

        self.mission_items.append(mission_item)

        return 0

    async def send_mission(self, reset: bool = True) -> int:
        """
        Send the mission to ardupilot.

        Args:
            reset (bool): Whether to reset the mission index to 0 after sending the mission, default is True.

        Returns:
            int: 0 if the mission was sent successfully, or an error code if there was an error.
        """

        # send mission count
        self.controller.send_mission_count(len(self.mission_items), self.type)

        # continuous loop to await mission request, or timeout
        start_time = time.time()
        while True:
            # await mission request
            seq = await self.controller.receive_mission_request()

            # verify seq is not an error
            if seq == self.controller.TIMEOUT_ERROR:
                return self.TIMEOUT_ERROR

            # send corresponding mission item
            self.controller.send_message(self.mission_items[seq].message)

            # check if all mission items have been sent
            if seq == len(self.mission_items) - 1:
                break

            # check for timeout
            if time.time() - start_time > self.MISSION_SEND_TIMEOUT:
                if self.controller.logger:
                    self.controller.logger.error(
                        f"[Mission] Mission send timeout after {self.MISSION_SEND_TIMEOUT} seconds"
                    )
                return self.TIMEOUT_ERROR

        # after sending all mission items, wait for mission acknowledgement
        response = await self.controller.receive_mission_ack()  # returns 0 if successful
        if response:
            return response  # propagate error code

        response = await self.controller.set_current_mission_index(0, reset=reset)
        if response:
            if self.controller.logger:
                self.controller.logger.critical("[Mission] Could not reset mission index.")
            return response

        return 0

    async def clear_mission(self) -> int:
        """
        Clear the mission from the vehicle.

        Returns:
            int: 0 if the mission was cleared successfully, or an error code if there was an error.
        """
        # send mission clear all
        self.controller.send_clear_mission()

        # await mission ack confirming mission was cleared
        response = await self.controller.receive_mission_ack()
        if response:
            if self.controller.logger:
                self.controller.logger.critical("[Mission] Could not clear mission.")
            return response

        return 0

    def __len__(self):
        return len(self.mission_items)
