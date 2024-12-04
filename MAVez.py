from pymavlink import mavutil

UNKNOWN_MODE = 1

class Controller:

    def __init__(self, connection_string='tcp:127.0.0.1:5762'):
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print(f"Heartbeat received from system (ID: {self.master.target_system})")
    

    def setMode(self, mode):
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