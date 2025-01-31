'''
flight_manager.py
version: 1.0.0

Theodore Tasman
2025-01-30
PSU UAS

This module is responsible for managing the flight of the drone.

Flight class - manages the flight plan of the drone.
'''

from flight_utils import Mission_Item, Mission, Coordinate
from mav_controller import Controller
from pymavlink import mavutil

class Flight:
    '''
        Manages the flight plan of the drone.
    '''
    
    def __init__(self, takeoff_mission_file, detect_mission_file, airdrop_mission_file, land_mission_file):
        '''
            Initialize the flight manager.
            takeoff_mission_file: str
            detect_mission_file: str
            airdrop_mission_file: str
            land_mission_file: str
        '''
        # Create a controller object
        self.controller = Controller()

        # create static missions
        self.takeoff_mission = Mission(self.controller)
        self.detect_mission = Mission(self.controller)
        self.land_mission = Mission(self.controller)

        # initialize variable missions
        self.airdrop_mission = None

        # Load the static missions from the files
        result = self.load_static_missions(takeoff_mission_file, detect_mission_file, land_mission_file)
        if result:
            raise Exception(f'Error in Flight initialization - failed to load static missions. Error code: {result}')

    
    def load_static_missions(self, takeoff_mission_file, detect_mission_file, land_mission_file):
        '''
            Load the static missions from the files.
            takeoff_mission_file: str
            detect_mission_file: str
            land_mission_file: str
            returns:
                0 if the missions were loaded successfully
                Mission error code if the missions failed to load (see flight_utils Mission class)
        '''
        # load takeoff mission
        result = self.takeoff_mission.load_mission_from_file(takeoff_mission_file)
        if result:
            print("Error loading takeoff mission")
            return result
        
        # load detect mission
        result = self.detect_mission.load_mission_from_file(detect_mission_file)
        if result:
            print("Error loading detect mission")
            return result

        # load land mission
        result = self.land_mission.load_mission_from_file(land_mission_file)
        if result:
            print("Error loading land mission")
            return result

        return 0

    def build_airdrop_mission(self, target_coordinate, airdrop_mission_file, target_index):
        '''
            Build the airdrop mission.
            target_coordinate: Coordinate
            airdrop_mission_file: str
            target_index: int
            returns:
                0 if the mission was built successfully
                Mission error code if the mission failed to build (see flight_utils Mission class)
        '''

        # Create a new mission object
        airdrop_mission = Mission(self.controller)

        # Load the mission from the file up to the target index
        result = airdrop_mission.load_mission_from_file(airdrop_mission_file, end=target_index)
        # verify that the mission was loaded successfully
        if result:
            return result

        # Create a target mission item
        target_mission_item = Mission_Item(
            frame=3,
            command=16,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            x=target_coordinate.latitude,
            y=target_coordinate.longitude,
            z=0
        )

        # Add the target mission item to the mission
        result = airdrop_mission.add_mission_item(target_mission_item)
        # verify that the mission item was added successfully
        if result:
            return result

        # Load the rest of the mission from the file
        result = airdrop_mission.load_mission_from_file(airdrop_mission_file, start=target_index)
        # verify that the mission was loaded successfully
        if result:
            return result
        
        return 0




