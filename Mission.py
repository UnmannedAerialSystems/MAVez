'''
Mission class

Ted Tasman
2025-01-30
PSU UAS

This class represents a mission for the drone.
'''

from .Mission_Item import Mission_Item # type: ignore
from .Coordinate import Coordinate # type: ignore
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

    def load_mission_from_file(self, filename, start=0, end=-1, first_seq=-1, overwrite=True):
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

        FILE_NOT_FOUND = 201
        FILE_EMPTY = 202
        START_OUT_OF_RANGE = 203
        END_OUT_OF_RANGE = 204

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

        # prevent re-loading of mission items to the same mission
        if overwrite:
            self.mission_items = []

        # slice out intended lines
        # if no end is specified, slice to the end of the file
        # index + 1 to skip the header
        if end == -1:
            lines = lines[start + 1:]
        else:
            lines = lines[start + 1:end + 1]

        for line in lines:
            # skip empty lines
            if line == '\n':
                continue

            parts = line.strip().split('\t')
            
            # jump seq ahead if first_seq is not -1
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
        # start the mission
        if self.type == 0: # if the mission is a waypoint mission
            print("Starting mission")
            self.controller.set_mode('AUTO') # set the mode to AUTO
            self.controller.start_mission(0, len(self.mission_items) - 1)

        return 0


    
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
    
    def __len__(self): 
        return len(self.mission_items)
    

    def validate_mission_file(filename): # this has to be in state actions so the filepaths match
        '''
            Validate a mission from a file.
            filename: str
            start: int
            end: int
            returns:
                0 if the mission was loaded successfully
                201 if the file was not found
                202 if the file was empty
        '''

        FILE_NOT_FOUND = 201
        FILE_EMPTY = 202

        try:
            with open(filename, 'r') as file:
                lines = file.readlines()
        except FileNotFoundError:
            print(f'File {filename} not found')
            return FILE_NOT_FOUND

        if len(lines) == 0:
            print('File is empty')
            return FILE_EMPTY

        for line in lines[1:]:
            # skip empty lines
            if line == '\n':
                continue

            parts = line.strip().split('\t')
            if len(parts) != 12:
                print(f'Invalid line: {line}')
                return FILE_EMPTY
        
        print(f'Verified {len(lines)} items from {filename}')
        return 0


def main():

    test_target = Coordinate(38.31567571033244, -76.55180243803832, alt=0)

    offset_target = test_target.offset_coordinate(100, 282)

    offset_target_2 = test_target.offset_coordinate(100, 282+180)

    print(test_target)
    print(offset_target)
    print(offset_target_2)


if __name__ == "__main__":
    main()