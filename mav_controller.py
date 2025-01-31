'''
mav_controller.py
version: 1.0.0

Theodore Tasman
2025-01-30
PSU UAS

This module is responsible for controlling the drone.

Controller class - facilitates communication with the drone.
'''

from pymavlink import mavutil

class Controller:
    '''
        Facilitates communication with the drone.

        Error codes:
            101 - timeout error
            111 - unknown mode
    '''

    # class wide error codes 
    # if an error is function specific, it will be defined in the function and documented in class docstring
    TIMEOUT_ERROR = 101


    TIMEOUT_DURATION = 5 # timeout duration in seconds

    def __init__(self, connection_string='tcp:127.0.0.1:5762'):
        '''
            Initialize the controller.
            connection_string: str
            verbose: bool
        '''

        self.master = mavutil.mavlink_connection(connection_string)

        response = self.master.wait_heartbeat(blocking=True, timeout=self.TIMEOUT_DURATION)

        if not response:
            raise Exception('Error in Controller initialization - no heartbeat received')


    def await_mission_request(self):
        '''
            Wait for a mission request from the drone.
            returns:
                0 if a mission request was received
                101 if the response timed out
        '''

        response = self.master.recv_match(type='MISSION_REQUEST', blocking=True, timeout=self.TIMEOUT_DURATION)
        if response:
            return 0
        else:
            return self.TIMEOUT_ERROR


    def await_mission_ack(self):
        '''
            Wait for a mission ack from the drone.
            returns:
                0 if a mission ack was received
                101 if the response timed out
        '''

        response = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=self.TIMEOUT_DURATION)
        if response:
            return 0
        else:
            return self.TIMEOUT_ERROR


    def send_message(self, message):
        '''
            Send a message to the drone. DOES NOT AWAIT RESPONSE.
            message: mavlink message
        '''

        self.master.mav.send(message)


    def send_mission_count(self, count, mission_type=0):
        '''
            Send the mission count to the drone. DOES NOT AWAIT RESPONSE.
            count: int

            Mission types:
            - 0: normal mission
            - 1: geofence
            - 2: rally points
            - 255: all mission types (only used in MISSION_CLEAR_ALL to clear all mission types)
        '''

        self.master.mav.mission_count_send(
            0, # target_system
            0, # target_component
            count, # count
            0 # mission_type
        )

    def set_mode(self, mode):
        '''
            Set the mode of the drone.
            mode: str
            returns:
                0 if the mode was set successfully
                111 if the mode is not recognized
        '''
        UNKNOWN_MODE = 111

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

        response = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=self.TIMEOUT_DURATION)
        if response:
            return 0
        else:
            return self.TIMEOUT_ERROR
        
    

    def arm(self, force=False):
        '''
            Arm the drone. If force is set to True, the drone will be armed regardless of the state of the drone.
            force: bool
            returns:
                0 if the drone was armed successfully
                1 if the response timed out
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

        response = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=self.TIMEOUT_DURATION)
        if response:
            return 0
        else:
            return self.TIMEOUT_ERROR
    

    def disarm(self, force=False):

        '''
            Disarm the drone. If force is set to True, the drone will be disarmed regardless of the state of the drone.
            force: bool
            returns:
                0 if the drone was disarmed successfully
                1 if the response timed out
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

        response = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=self.TIMEOUT_DURATION)
        if response:
            return 0
        else:
            return self.TIMEOUT_ERROR


    def enable_geofence(self):
        '''
            Enable the geofence.
            returns:
                0 if the geofence was enabled successfully
                1 if the response timed out
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

        response = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=self.TIMEOUT_DURATION)
        if response:
            return 0
        else:
            return self.TIMEOUT_ERROR


    def disable_geofence(self, floor_only=False):
        '''
            Disable the geofence.
            floor_only: bool - if set to True, only the floor geofence will be disabled
            returns:
                0 if the geofence was disabled successfully
                1 if the response timed out
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

        response = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=self.TIMEOUT_DURATION)
        if response:
            return 0
        else:
            return self.TIMEOUT_ERROR

