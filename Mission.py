'''
Mission class

Ted Tasman
2025-01-30
PSU UAS

This class represents a mission for the drone.
'''

from Mission_Item import Mission_Item
from Coordinate import Coordinate
import time

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

    def load_mission_from_file(self, filename, start=0, end=-1, first_seq=-1):
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

            if first_seq != -1:
                seq = first_seq
                first_seq += 1
            else:
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

    
    def wait_for_channel_input(self, channel, value, timeout=10, wait_time=120):
        '''
        wait for a specific rc channel to reach a specific value
        channel: int
        value: int
        timeout: int
        returns:
            0 if the rc channel reached the value
            101 if the timeout was reached
        '''
        latest_value = -float('inf')
        start_time = time.time()
        while latest_value != value:
            response = self.controller.await_channel_input(channel, timeout)

            if response == self.controller.TIMEOUT_ERROR:
                return response
            
            latest_value = response
            if latest_value > value:
                print(f'Channel {channel} value: {latest_value}')
                return 0

            # check for timeout
            if time.time() - start_time > timeout:
                print(f'Channel was not set to {value} within {timeout} seconds')
                return self.TIMEOUT_ERROR

    
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
    

def main():

    test_target = Coordinate(38.31567571033244, -76.55180243803832, alt=0)

    offset_target = test_target.offset_coordinate(100, 282)

    offset_target_2 = test_target.offset_coordinate(100, 282+180)

    print(test_target)
    print(offset_target)
    print(offset_target_2)


if __name__ == "__main__":
    main()