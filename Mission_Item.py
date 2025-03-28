'''
Mission_Item class

Ted Tasman
2025-01-30
PSU UAS

This class represents a mission item for the drone.
It provides methods to initialize the mission item and to convert it to a message.
'''

from pymavlink import mavutil

class Mission_Item:

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
        return f'Seq: {self.seq} \nFrame: {self.frame}\nCommand: {self.command}\nCurrent: {self.current}\nAuto Continue: {self.auto_continue}\nX: {self.x}\nY: {self.y}\nZ: {self.z}\nType: {self.type}\nParam1: {self.param1}\nParam2: {self.param2}\nParam3 {self.param3}\nParam4 {self.param4}'

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