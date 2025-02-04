'''
flight_utils.py
version: 1.0.2

Theodore Tasman
2025-01-30
PSU UAS

This module contains utility classes for managing flights.

Coordinate class - represents a coordinate in latitude, longitude, and altitude.
Mission_Item class - represents a mission item for the drone.
Mission class - represents a mission for the drone.
'''

from pymavlink import mavutil

class Coordinate:
    '''
        Represents a coordinate in latitude, longitude, and altitude.
    '''

    def __init__(self, lat, lon, alt, dms=False, use_int=True):
        '''
            Initialize the coordinate.
            lat: float | str if dms is True
            lon: float | str if dms is True
            alt: float
            dms: bool - if set to True, the coordinates are in degrees, minutes, seconds
            integer: bool - if set to True, the coordinates are stored as integers
        '''

        # if dms is True, convert the coordinates to decimal degrees
        if dms:
            self.lat = self.dms_to_dd(lat)
            self.lon = self.dms_to_dd(lon)
        # otherwise, use the coordinates as they are
        else:
            self.lat = lat
            self.lon = lon

        # alt is always used as it is
        self.alt = alt

        # if integer is True, convert the coordinates to integers
        if use_int:
            self.lat = int(self.lat * 1e7)
            self.lon = int(self.lon * 1e7)
            self.alt = int(self.alt)
    
    def __str__(self):
        return f'{self.lat} {self.lon} {self.alt}'
    
    __repr__ = __str__

    def dms_to_dd(self, dms):
        '''
            Convert degrees, minutes, seconds to decimal degrees.
            dms: float
            returns:
                dd: float
        '''
        dd = dms[0] + dms[1] / 60 + dms[2] / 3600
        return dd


class Mission_Item:
    '''
        Represents a mission item for the drone.
    '''

    def __init__(self, seq, frame, command, current, auto_continue, coordinate, type=0, param1=0, param2=0, param3=0, param4=0):
        '''
            Initialize the mission item.
            seq: int
            frame: int
            command: int
            current: int
            auto_continue: int
            x: float
            y: float
            z: float
            param1: float
            param2: float
            param3: float
            param4: float
        '''
        self.seq = int(seq)
        self.frame = int(frame)
        self.command = int(command)
        self.current = int(current)
        self.auto_continue = int(auto_continue)
        self.x = coordinate.lat
        self.y = coordinate.lon
        self.z = coordinate.alt
        self.param1 = int(param1)
        self.param2 = int(param2)
        self.param3 = int(param3)
        self.param4 = int(param4)
        self.type = type
    

    def __str__(self):
        return f'{self.seq} {self.frame} {self.command} {self.current} {self.auto_continue} {self.x} {self.y} {self.z} {self.type} {self.param1} {self.param2} {self.param3} {self.param4}'

    __repr__ = __str__

    @property
    def message(self):
        '''
            Get the message to be sent to the drone.
            returns:
                message: pymavlink.mavutil.mavlink
        '''
        message = mavutil.mavlink.MAVLink_mission_item_int_message(
            0, # target_system
            0, # target_component
            self.seq, # seq
            self.frame, # frame
            self.command, # command
            self.current, # current
            self.auto_continue, # auto continue
            self.param1, # param1
            self.param2, # param2
            self.param3, # param3
            self.param4, # param4
            self.x, # x
            self.y, # y
            self.z, # z
            self.type # type
        )
        return message


class Mission:
    '''
        Represents a mission for the drone.

        Error codes:
            201 - file not found (load_mission_from_file)
            202 - file empty (load_mission_from_file)
            203 - start index out of range (load_mission_from_file)
            204 - end index out of range (load_mission_from_file)
    '''

    # class wide error codes 
    # if an error is function specific, it will be defined in the function and documented in class docstring
    TIMEOUT_ERROR = 101


    def __init__(self, controller, type=0):
        '''
            Initialize the mission.
            controller: Controller
        '''
        self.controller = controller
        self.type = type
        self.mission_items = []
    
    def __str__(self):
        output = ''
        for mission_item in self.mission_items:
            output += f'{mission_item}\n'

        return output
    
    __repr__ = __str__

    def decode_error(self, error_code):

        error_codes = {
            201: "\nFILE NOT FOUND ERROR (201)\n",
            202: "\nEMPTY FILE ERROR (202)\n",
            203: "\nINVALID START INDEX ERROR (203)\n",
            204: "\nINVALID END INDEX ERROR (204)\n",
        }

        return error_codes.get(error_code, f"\nUNKNOWN ERROR ({error_code})\n")

    def load_mission_from_file(self, filename, start=0, end=-1):
        '''
            Load a mission from a file.
            filename: str
            start: int
            end: int
            returns:
                0 if the mission was loaded successfully
                201 if the file was not found
                202 if the file was empty
                203 if the start index was out of range
                204 if the end index was out of range
        '''

        FILE_NOT_FOUND = 1
        FILE_EMPTY = 2
        START_OUT_OF_RANGE = 3
        END_OUT_OF_RANGE = 4

        try:
            with open(filename, 'r') as file:
                lines = file.readlines()
        except FileNotFoundError:
            print(f'File {filename} not found')
            return FILE_NOT_FOUND

        if len(lines) == 0:
            print('File is empty')
            return FILE_EMPTY
        
        elif start >= len(lines) - 1:
            print('Start index out of range')
            return START_OUT_OF_RANGE
        
        elif end != -1 and end >= len(lines) - 1:
            print('End index out of range')
            return END_OUT_OF_RANGE

        # slice out intended lines
        # if no end is specified, slice to the end of the file
        # index + 1 to skip the header
        if end == -1:
            lines = lines[start + 1:]
        else:
            lines = lines[start + 1:end + 1]

        for line in lines:
            parts = line.strip().split('\t')
            seq = int(parts[0])
            current = int(parts[1])
            frame = int(parts[2])
            command = int(parts[3])
            param1 = float(parts[4])
            param2 = float(parts[5])
            param3 = float(parts[6])
            param4 = float(parts[7])
            x = float(parts[8])
            y = float(parts[9])
            z = float(parts[10])
            auto_continue = int(parts[11])

            item_coordinate = Coordinate(x, y, z)
            mission_item = Mission_Item(seq, frame, command, current, auto_continue, item_coordinate, self.type, param1, param2, param3, param4)
            self.mission_items.append(mission_item)
        
        print(f'Loaded {len(self.mission_items)} mission items from {filename}')
        return 0


    def save_mission_to_file(self, filename):
        '''
            Save the mission to a file.
            filename: str
            returns:
                0 if the mission was saved successfully
        '''
        with open(filename, 'w') as file:
            file.write('QGC WPL 110\n')
            for mission_item in self.mission_items:
                file.write(f'{mission_item.seq}\t{mission_item.current}\t{mission_item.frame}\t{mission_item.command}\t{mission_item.param1}\t{mission_item.param2}\t{mission_item.param3}\t{mission_item.param4}\t{mission_item.x}\t{mission_item.y}\t{mission_item.z}\t{mission_item.auto_continue}\n')

        return 0

    def add_mission_item(self, mission_item):
        '''
            Add a mission item to the mission.
            mission_item: Mission_Item
            returns:
                0 if the mission item was added successfully
        '''

        self.mission_items.append(mission_item)

        return 0
    

    def send_mission(self):
        '''
            Send the mission to the drone.
            returns:
                0 if the mission was sent successfully
                101 if the response timed out
        '''

        # TODO: add support for mission types

        # send mission count
        self.controller.send_mission_count(len(self.mission_items), self.type)

        # loop through mission items and send them
        for mission_item in self.mission_items:

            # await mission request
            response = self.controller.await_mission_request()
            if response:
                return response
            
            # send next mission item
            print(mission_item.seq)
            self.controller.send_message(mission_item.message)
        
        print("Awaiting mission ack")
        # await mission ack confirming mission was received 
        response = self.controller.await_mission_ack()
        if response:
            return response
        
        print("Mission sent successfully")
        return 0


    def wait_for_waypoint_reached(self, target, timeout=10):
        '''
            Wait for the drone to reach the current waypoint.
            timeout: int
            returns:
                0 if the waypoint was reached
                101 if the timeout was reached
        '''
        latest_waypoint = -1
        
        while latest_waypoint < target:
            response = self.controller.await_mission_item_reached(timeout)

            if response == self.controller.TIMEOUT_ERROR:
                return response

            latest_waypoint = response
            print(f'Waypoint reached: {latest_waypoint}')

    
    def clear_mission(self):
        '''
            Clear the mission from the drone.
            returns:
                0 if the mission was cleared successfully
                101 if the response timed out
        '''
        # send mission clear all
        print("Clearing mission")
        self.controller.send_clear_mission()

        # await mission ack confirming mission was cleared
        response = self.controller.await_mission_ack()
        if response:
            return response
        
        return 0
    
    def get_length(self): 
        return len(self.mission_items)