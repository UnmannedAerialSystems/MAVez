from pymavlink import mavutil

UNKNOWN_MODE = 1

TO = 22
LD = 21
WP = 16


class Controller:

    def __init__(self, connection_string='tcp:127.0.0.1:5762', verbose=False):
        '''
            Initialize the controller.
            connection_string: str
            verbose: bool
        '''

        self.master = mavutil.mavlink_connection(connection_string)

        if verbose:
            print("Waiting for heartbeat...")

        self.master.wait_heartbeat()

        if verbose:
            print(f"Heartbeat received from system (ID: {self.master.target_system})")
    

    def decode_error(self, error_code):
        '''
            Decode the error code from the response of the drone.
            error_code: int
            returns:
                str
        '''
        if error_code == 1:
            return 'UNKNOWN CONTROL MODE'


    def set_mode(self, mode):
        '''
            Set the mode of the drone.
            mode: str
            returns:
                response from the drone if the mode was set successfully
                UNKNOWN_MODE if the mode is not recognized
        '''

        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            return UNKNOWN_MODE
        
        mode_id = self.master.mode_mapping()[mode]
        message = self.master.mav.command_long_encode(
            0, # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, # command
            0, # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, # param1
            mode_id, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0 # param7
        )

        self.master.mav.send(message)

        response = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        return response
    

    def arm(self, force=False):
        '''
            Arm the drone. If force is set to True, the drone will be armed regardless of the state of the drone.
            force: bool
            returns:
                response from the drone if the drone was armed successfully
        '''
        message = self.master.mav.command_long_encode(
            0, # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
            0, # confirmation
            1, # param1
            21196 if force else 0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0 # param7
        )

        self.master.mav.send(message)

        response = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        return response
    

    def disarm(self, force=False):

        '''
            Disarm the drone. If force is set to True, the drone will be disarmed regardless of the state of the drone.
            force: bool
            returns:
                response from the drone if the drone was disarmed successfully
        '''

        message = self.master.mav.command_long_encode(
            0, # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # command
            0, # confirmation
            0, # param1
            21196 if force else 0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0 # param7
        )

        self.master.mav.send(message)

        response = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        return response


    def enable_geofence(self):
        '''
            Enable the geofence.
            returns:
                response from the drone if the geofence was enabled successfully
        '''
        message = self.master.mav.command_long_encode(
            0, # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, # command
            0, # confirmation
            1, # param1
            0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0 # param7
        )

        self.master.mav.send(message)
        response = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        return response


    def disable_geofence(self, floor_only=False):
        '''
            Disable the geofence.
            floor_only: bool - if set to True, only the floor geofence will be disabled
            returns:
                response from the drone if the geofence was disabled successfully
        '''
        message = self.master.mav.command_long_encode(
            0, # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE, # command
            0, # confirmation
            2 if floor_only else 0, # param1
            0, # param2
            0, # param3
            0, # param4
            0, # param5
            0, # param6
            0 # param7
        )

        self.master.mav.send(message)
        response = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        return response


    def go_to_override(self, lat, lon, alt, yaw=float('nan'), hold=False, after=0):
        '''
            Send the drone to a waypoint overriding the current mission.
            lat: float
            lon: float
            alt: float
            yaw: float - NaN for unchanged
            hold: bool - if set to True, the drone will hold at the waypoint
            after: int - if hold is set to True, 0 for specified position, 1 for current position
            returns:
                response from the drone if the drone was sent to the waypoint successfully
        '''
        message = self.master.mav.command_long_encode(
            0, # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE, # command
            0, # confirmation
            0,#0 if hold else 1, # param1
            0,#2 if after else 3, # param2
            0,#0, # param3 - 0 for global frame
            0,
            0, # param5
            0, # param6
            0, # param7
        )

        self.master.mav.send(message)
        response = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        print(response)

        message = self.master.mav.command_long_encode(
            0, # target_system
            0, # target_component
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, # command
            0, # confirmation
            -1, # param1
            1, # param2
            20, # param3
            yaw, # param4
            lat, # param5
            lon, # param6
            alt # param7
        )

        self.master.mav.send(message)
        response = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        print(response)


    


class Mission_Item:

    def __init__(self, seq, frame, command, current, autocontinue, x, y, z, param1=0, param2=0, param3=0, param4=0):
        '''
            Initialize the mission item.
            seq: int
            frame: int
            command: int
            current: int
            autocontinue: int
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
        self.autocontinue = int(autocontinue)
        self.x = int(x * 1e7)
        self.y = int(y * 1e7)
        self.z = int(z)
        self.param1 = int(param1)
        self.param2 = int(param2)
        self.param3 = int(param3)
        self.param4 = int(param4)
    
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
            self.autocontinue, # autocontinue
            self.param1, # param1
            self.param2, # param2
            self.param3, # param3
            self.param4, # param4
            self.x, # x
            self.y, # y
            self.z # z
        )
        return message
    

    def __str__(self):
        return f'{self.seq} {self.frame} {self.command} {self.current} {self.autocontinue} {self.x} {self.y} {self.z} {self.param1} {self.param2} {self.param3} {self.param4}'

    __repr__ = __str__

class Mission:

    def __init__(self, controller):
        '''
            Initialize the mission.
            controller: Controller
        '''
        self.controller = controller
        self.mission_items = []
    
    def __str__(self):
        output = ''
        for mission_item in self.mission_items:
            output += f'{mission_item}\n'

        return output
    
    __repr__ = __str__

    def load_mission_from_file(self, filename):
        '''
            Load a mission from a file.
            filename: str
        '''
        with open(filename, 'r') as file:
            lines = file.readlines()
            lines = lines[1:]
            for line in lines:
                parts = line.split('\t')
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
                autocontinue = int(parts[11])

                mission_item = Mission_Item(seq, frame, command, current, autocontinue, x, y, z, param1, param2, param3, param4)
                self.mission_items.append(mission_item)

        print(self)

    def save_mission_to_file(self, filename):
        '''
            Save the mission to a file.
            filename: str
        '''
        with open(filename, 'w') as file:
            file.write('QGC WPL 110\n')
            for mission_item in self.mission_items:
                file.write(f'{mission_item.seq}\t{mission_item.current}\t{mission_item.frame}\t{mission_item.command}\t{mission_item.param1}\t{mission_item.param2}\t{mission_item.param3}\t{mission_item.param4}\t{mission_item.x}\t{mission_item.y}\t{mission_item.z}\t{mission_item.autocontinue}\n')



    def add_mission_item(self, mission_item):
        '''
            Add a mission item to the mission.
            mission_item: Mission_Item
        '''
        self.mission_items.append(mission_item)
    

    def send_mission(self):
        '''
            Send the mission to the drone.
            returns:
                response from the drone if the mission was sent successfully
        '''
        print('Sending mission...')
        print('Sending mission count...')
        self.controller.master.mav.mission_count_send(
            0, # target_system
            0, # target_component
            len(self.mission_items), # count
            0 # mission_type
        )

        for mission_item in self.mission_items:
            print(f'Awaiting request for mission item {mission_item.seq}')
            response = self.controller.master.recv_match(type='MISSION_REQUEST', blocking=True)
            print(response)
            print(f'Sending mission item {mission_item.seq}')
            self.controller.master.mav.send(mission_item.message)
        
        response = self.controller.master.recv_match(type='MISSION_ACK', blocking=True)
        print(response)
        
        return response
    

    def clear_mission(self):
        '''
            Clear the mission from the drone.
            returns:
                response from the drone if the mission was cleared successfully
        '''
        message = self.controller.master.mav.mission_clear_all_encode(
            0, # target_system
            0 # target_component
        )

        self.controller.master.mav.send(message)

        response = self.controller.master.recv_match(type='MISSION_ACK', blocking=True)
        return response



def main():
    controller = Controller()
    mission = Mission(controller)

    # input('Press Enter to load mission from file')
    # mission.load_mission_from_file('test_mission.txt')

    # input('Press Enter to send mission')
    # mission.send_mission()

    # input('Press Enter to set mode to AUTO')
    # controller.set_mode('AUTO')

    # input('Press Enter to arm')
    # controller.arm()

    input('Send override location')
    controller.go_to_override(38, -76, 100, yaw=0)




if __name__ == '__main__':
    main()