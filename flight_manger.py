'''
flight_manager.py

Theodore Tasman
2025-01-30
PSU UAS

This module is responsible for managing the flight of the drone.

Flight class - manages the flight plan of the drone.

SITL Start Command: 
python3 ../MAVLink/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane --console --map --custom-location 38.31527628,-76.54908330,40,282.5

'''

# . imports are relative to the current file
from .Coordinate import Coordinate # type: ignore
from .Mission import Mission # type: ignore
from .Mission_Item import Mission_Item # type: ignore
from .mav_controller import Controller # type: ignore
from pymavlink import mavutil # type: ignore
import time
import sys
import os
# Add the parent directory to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
#from CameraModule import camera_emulator # type: ignore
#from ObjectDetection import lion_sight_emulator # type: ignore

class Flight(Controller):
    '''
        Manages the flight plan of the drone.
    '''
    
    PREFLIGHT_CHECK_ERROR = 301
    DETECT_LOAD_ERROR = 302
    AIRDROP_NOT_BUILT_ERROR = 303

    def __init__(self, connection_string='tcp:127.0.0.1:5762', logger=None):
        '''
            Initialize the flight manager.
        '''
        # Initialize the controller
        super().__init__(connection_string, logger=logger)

        # initialize preflight check
        self.preflight_check_done = False

        # create missions
        self.takeoff_mission = Mission(self)
        self.detect_mission = Mission(self)
        self.land_mission = Mission(self)
        self.airdrop_mission = Mission(self)
        self.geofence = Mission(self, type=1) # type 1 is geofence

        # initialize mission list
        self.mission_list = [self.takeoff_mission] # TODO: takeoff mission


    def decode_error(self, error_code):
        '''
            Decode an error code.
            error_code: int
            returns: str
        '''

        errors_dict = {
            101: "\nTIMEOUT ERROR (101)\n",
            111: "\nUNKNOWN MODE ERROR (102)\n",
            301: "\nPREFLIGHT CHECK ERROR (301)\n",
            302: "\nDETECT LOAD ERROR (302)\n"
        }

        return errors_dict.get(error_code, f"UNKNOWN ERROR ({error_code})")


    def takeoff(self, takeoff_mission_file):
        '''
            Takeoff the drone. Preflight check must be done first.
            returns:
                0 if the drone took off successfully
                Mission error code if the drone failed to takeoff (see flight_utils Mission class)
        '''

        # verify preflight check
        if not self.preflight_check_done:
            return self.PREFLIGHT_CHECK_ERROR

        # Load the takeoff mission from the file
        response = self.takeoff_mission.load_mission_from_file(takeoff_mission_file)

        # verify that the mission was loaded successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Takeoff failed, mission not loaded")
            return response

        # send the takeoff mission
        response = self.takeoff_mission.send_mission()

        # verify that the mission was sent successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Takeoff failed, mission not sent")
            return response
        
        # wait for mission to be fully received
        # Countdown from 5
        if self.logger:
            self.logger.info("[Flight] Takeoff in 5 seconds")
        for i in range(5, 0, -1):
            time.sleep(1)

        # set the mode to AUTO
        response = self.set_mode('AUTO')

        # verify that the mode was set successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Takeoff failed, mode not set to AUTO")
            return response
        
        # arm the drone
        response = self.arm()

        # verify that the drone was armed successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Takeoff failed, drone not armed")
            return response
        
        return 0
        

    def build_airdrop_mission(self, target_coordinate, airdrop_mission_file, target_index, altitude, drop_count):
        '''
            Build the airdrop mission.
            target_coordinate: Coordinate
            airdrop_mission_file: str
            target_index: int
            returns:
                0 if the mission was built successfully
                Mission error code if the mission failed to build (see flight_utils Mission class)
        '''

        # Load the mission from the file up to the target index
        result = self.airdrop_mission.load_mission_from_file(airdrop_mission_file, end=target_index)
        
        # verify that the mission was loaded successfully
        if result:
            return result

        # Set the target coordinate altitude
        target_coordinate.alt = altitude

        # calculate the buffer coordinates
        # entry_coordinate = target_coordinate.offset_coordinate(buffer_distance, heading + 180) # entry point is opposite of target
        # exit_coordinate = target_coordinate.offset_coordinate(buffer_distance, heading)

        # # Create the entry mission item
        # entry_mission_item = Mission_Item(
        #     seq=target_index,
        #     frame=3, 
        #     command=16, 
        #     current=0, 
        #     auto_continue=1, 
        #     coordinate=entry_coordinate, 
        #     type=0, 
        #     param1=0, 
        #     param2=0, 
        #     param3=0, 
        #     param4=0
        # )


        # Create drop script trigger
        drop_script_trigger = Mission_Item(
            seq=target_index,
            frame=3, 
            command=217,  # MAV_CMD_DO_SEND_SCRIPT_MESSAGE
            current=0, 
            auto_continue=1, 
            coordinate=Coordinate(0, 0, 0),  # unused coordinate
            type=0, 
            param1=17491,  # MAVLink message ID for the script message
            param2=drop_count % 2 + 1,  # 1 for even drop, 2 for odd drop (0 indexed)
            param3=0,  # unused
            param4=0   # unused
        )

        # Create the target mission item
        target_mission_item = Mission_Item(
            seq=target_index + 1,
            frame=3,
            command=16,
            current=0, 
            auto_continue=1,
            coordinate=target_coordinate,
            type=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0
        )
        
        # # Create the exit mission item
        # exit_mission_item = Mission_Item(
        #     seq=target_index + 3,
        #     frame=3, 
        #     command=16, 
        #     current=0, 
        #     auto_continue=1, 
        #     coordinate=exit_coordinate, 
        #     type=0, 
        #     param1=0, 
        #     param2=0, 
        #     param3=0, 
        #     param4=0
        # )
        
        # Add the entry mission item to the mission
        # result = self.airdrop_mission.add_mission_item(entry_mission_item)
        # # verify that the mission item was added successfully
        # if result:
        #     if self.logger:
        #         self.logger.critical(f"[Flight] Failed to append entry mission item, {result}")
        #     return result
        # if self.logger:
        #     self.logger.info("[Flight] Appended entry mission item")

        # Add the drop script trigger to the mission
        result = self.airdrop_mission.add_mission_item(drop_script_trigger)
        # verify that the mission item was added successfully
        if result:
            if self.logger:
                self.logger.critical(f"[Flight] Failed to append drop script trigger, {result}")
            return result
        if self.logger:
            self.logger.info("[Flight] Appended drop script trigger")

        # Add the target mission item to the mission
        result = self.airdrop_mission.add_mission_item(target_mission_item)
        # verify that the mission item was added successfully
        if result:
            if self.logger:
                self.logger.critical(f"Failed to append target mission item, {result}")
            return result
        if self.logger:
            self.logger.info("[Flight] Appended target mission item")
        
        # # Add the exit mission item to the mission
        # result = self.airdrop_mission.add_mission_item(exit_mission_item)
        # # verify that the mission item was added successfully
        # if result:
        #     if self.logger:
        #         self.logger.critical(f"[Flight] Failed to append exit mission item, {result}")
        #     return result
        
        # if self.logger:
        #     self.logger.info("[Flight] Appended exit mission item")

        # Load the rest of the mission from the file, don't overwrite the existing mission items
        result = self.airdrop_mission.load_mission_from_file(airdrop_mission_file, start=target_index, first_seq=target_index + 2, overwrite=False)
        # verify that the mission was loaded successfully
        if result:
            if self.logger:
                self.logger.critical(f"[Flight] Failed to load airdrop mission from file, {result}")
            return result
        if self.logger:
            self.logger.info(f"[Flight] Airdrop mission built successfully with {len(self.airdrop_mission)} items")
            self.logger.info(f"[Flight] Airdrop mission:\n{self.airdrop_mission}")
        

        return 0
    

    def append_airdrop_mission(self):
        '''
            Append the airdrop mission to the mission list.
        '''
        # Check if the airdrop mission is empty
        if len(self.airdrop_mission) == 0:
            if self.logger:
                self.logger.critical("[Flight] Airdrop mission not built, cannot append to mission list")
            return self.AIRDROP_NOT_BUILT_ERROR

        self.mission_list.append(self.airdrop_mission)

        if self.logger:
            self.logger.info(f"[Flight] Appended airdrop mission to mission list")
        return 0
    
    # DEPRECATED just use append_mission
    def append_detect_mission(self, detect_mission_file=None):
        '''
            Append the detect mission to the mission list.
        '''
        if self.detect_mission.get_length() == 0:
            
            # Check if file is provided
            if detect_mission_file:
                response = self.detect_mission.load_mission_from_file(detect_mission_file)

                # verify that the mission was loaded successfully
                if response:
                    return response
                
            # if no file provided, return error
            else:
                return self.DETECT_LOAD_ERROR


        self.mission_list.append(self.detect_mission)

        return 0


    def append_mission(self, filename):
        '''
            Append a mission to the mission list.
            mission: Mission
        '''
        # Load the mission from the file
        mission = Mission(self)
        result = mission.load_mission_from_file(filename)

        if result:
            if self.logger:
                self.logger.critical(f"[Flight] Could not append mission.")
            return result
        
        if self.logger:
            self.logger.info(f"[Flight] Appended mission from {filename} to mission list")
        self.mission_list.append(mission)


    def wait_for_waypoint_reached(self, target, timeout=30):
        '''
            Wait for the drone to reach the current waypoint.
            timeout: int
            returns:
                0 if the waypoint was reached
                101 if the timeout was reached
        '''
        latest_waypoint = -1
        
        if self.logger:
            self.logger.info(f"[Flight] Waiting for waypoint {target} to be reached")

        while latest_waypoint < target:
            response = self.await_mission_item_reached(timeout)

            if response == self.TIMEOUT_ERROR:
                return response

            latest_waypoint = response
            
        
        if self.logger:
            self.logger.info(f"[Flight] Waypoint {target} reached")
        return 0


    def wait_and_send_next_mission(self):
        '''
            Waits for the last waypoint to be reached,
            clears the mission, sends the next mission,
            sets mode to auto.
            target_index: int
            returns:
                0 if the mission was sent successfully
                101 if the response timed out
        '''
        # Get the current mission
        current_mission = self.mission_list.pop(0)

        # if the mission list is empty, return
        if len(self.mission_list) == 0:
            if self.logger:
                self.logger.info(f"[Flight] No more missions in list, landing")
            #next_mission = self.land_mission
            return 0
        
        # otherwise, set the next mission to the next mission in the list
        else:
            if self.logger:
                self.logger.info(f"[Flight] Queuing next mission in list of {len(self.mission_list)} missions")
            next_mission = self.mission_list[0]

        # calculate the target index
        target_index = len(current_mission) - 1

        # Wait for the target index to be reached
        response = self.wait_for_waypoint_reached(target_index, 60)

        # verify that the response was received
        if response == self.TIMEOUT_ERROR:
            if self.logger:
                self.logger.critical(f"[Flight] Failed to wait for next mission.")
            return response

        # Clear the mission
        response = current_mission.clear_mission()
        if response:
            if self.logger:
                self.logger.critical(f"[Flight] Failed to send next mission.")
            return response

        # Send the next mission
        result = next_mission.send_mission()
        if result:
            if self.logger:
                self.logger.critical(f"[Flight] Failed to send next mission.")
            return result

        # set the mode to AUTO
        response = self.set_mode('AUTO')

        # verify that the mode was set successfully
        if response:
            if self.logger:
                self.logger.critical(f"[Flight] Failed to send next mission.")
            return response

        if self.logger:
            self.logger.info(f"[Flight] Next mission sent")
        return result
    
    
    def wait_for_landed(self, timeout=60):
        '''
            Wait for the drone to land.
            returns:
                0 if the drone landed successfully
                nonzero if the drone failed to land
        '''
        landing_status = -1

        # start receiving landing status
        response = self.set_message_interval(message_type=245, interval = 1e6) # 245 is landing status (EXTENDED_SYS_STATE), 1e6 is 1 second
        if response:
            if self.logger:
                self.logger.critical("[Flight] Failed waiting for landing.")
            return response

        # wait for landing status to be landed
        start_time = time.time()
        while landing_status != 1: # 1 for landed, 2 for in air, 3 for taking off, 4 for currently landing, 0 for unknown

            # check for timeout
            if time.time() - start_time > timeout:
                response = self.TIMEOUT_ERROR
                if self.logger:
                    self.logger.error("[Flight] Timed out waiting for landing.")
                return response

            # get the landing status
            response = self.receive_landing_status()

            # verify that the response was received
            if response == self.TIMEOUT_ERROR:
                if self.logger:
                    self.logger.error("[Flight] Failed waiting for landing.")
                return response

            landing_status = response
        
        # stop receiving landing status
        response = self.disable_message_interval(message_type=245) # 245 is landing status (EXTENDED_SYS_STATE)
        if response:
            if self.logger:
                self.logger.error("[Flight] Error waiting for landing.")
            return response
        
        return 0


    def preflight_check(self, land_mission_file, geofence_file, home_coordinate=Coordinate(0, 0, 0)):
        '''
            Perform a preflight check. On success, set preflight_check_done to True.
            land_mission_file: str
            geofence_file: str
            home_coordinate: Coordinate (optional, if not provided, the current position will be used)
            returns:
                0 if the preflight check passed
                nonzero if the preflight check failed
        '''
        
        # Set home location
        response = self.set_home(home_coordinate)

        # verify that the home location was set successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Preflight check failed, home location not set")
            return response
        
        # load geofence
        response = self.geofence.load_mission_from_file(geofence_file)

        # verify that the geofence was loaded successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Preflight check failed, geofence not loaded")
            return response
        
        # send geofence
        response = self.geofence.send_mission()

        # verify that the geofence was sent successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Preflight check failed, geofence not sent")
            return response
        
        # load land mission
        response = self.land_mission.load_mission_from_file(land_mission_file)

        # verify that the land mission was loaded successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Preflight check failed, land mission not loaded")
            return response
        
        # enable geofence
        response = self.enable_geofence()

        # verify that the geofence was enabled successfully
        if response:
            if self.logger:
                self.logger.critical("[Flight] Preflight check failed, geofence not enabled")
            return response
        
        # set preflight check done
        self.preflight_check_done = True

        if self.logger:
            self.logger.info("[Flight] Preflight check passed")
        return 0
    
    def jump_to_next_mission_item(self):
        '''
            Jump to the next mission item.
            returns:
                0 if the mission item was jumped to successfully
                nonzero if the mission item failed to jump to
        '''

        if self.logger:
            self.logger.info("[Flight] Waiting for current mission index")
        # wait for the current mission target to be received (should be broadcast by default)
        response = self.await_current_mission_index()
        if response == self.TIMEOUT_ERROR:
            return response

        # jump to the next mission item
        response = self.set_current_mission_index(response + 1)
        if response:
            return response
        
        return 0
    

    def wait_for_channel_input(self, channel, value, timeout=10, wait_time=120, value_tolerance=0):
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

        # set the channel to be received
        channel = f'chan{channel}_raw'

        if self.logger:
            self.logger.info(f"[Flight] Waiting for channel {channel} to be set to {value}")
        # only wait for the channel to be set for a certain amount of time
        while time.time() - start_time < wait_time:

            # get channel inputs
            response = self.receive_channel_input(timeout)

            # verify that the response was received
            if response == self.TIMEOUT_ERROR:
                if self.logger:
                    self.logger.critical("[Flight] Failed waiting for channel input.")
                return response
            
            # channel key is 'chanX_raw' where X is the channel number
            latest_value = getattr(response, channel)

            #print(f'Latest value: {latest_value}')
            # check if the value is within the tolerance range
            if latest_value > value - value_tolerance and latest_value < value + value_tolerance:
                if self.logger:
                    self.logger.info(f"[Flight] Channel {channel} set to {latest_value}")
                return 0

        if self.logger:
            self.logger.critical(f"[Flight] Timed out waiting for channel {channel} to be set to {value}")
        return self.TIMEOUT_ERROR
    

    def get_altitude(self):
        '''
            Get the altitude of the drone.
            returns:
                altitude: float
        '''
        # get the altitude
        if self.logger:
            self.logger.info("[Flight] Getting altitude")

        response = self.receive_altitude()

        # verify that the response was received
        if response == self.TIMEOUT_ERROR:
            if self.logger:
                self.logger.critical("[Flight] Failed to get altitude.")
            return response
        
        return response
        


    

        
            

def main():

    # Create a flight object
    flight = Flight()

    # Create a lion sight emulator object
    lion_sight = lion_sight_emulator.LionSight(predetermined_coordinate=[Coordinate(lat=(38,18,56), lon=(-76,33,6), alt=0, dms=True)])

    # Create a camera emulator object
    camera = camera_emulator.Camera(
                    backdrop_file='../CameraModule/runway_smaller.png',
                    resolution=(1440, 1080),
                    area_y_offset=400,
                    area_height=800,
                    area_width=6400,
                    num_targets=4,
                    targets_folder='../CameraModule/targets_small',
                    y_offset=250
    )

    
    # Perform preflight check
    response = flight.preflight_check('landing_mission.txt', 'geofence.txt')
    if response:
        print(flight.decode_error(response))
        return

    # Append detect mission
    response = flight.append_detect_mission('detect_mission.txt')
    if response:
        print(flight.decode_error(response))
        return

    # Takeoff the drone
    response = flight.takeoff('takeoff_mission.txt')
    if response:
        print(flight.decode_error(response))
        return

    # wait and run next mission
    response = flight.wait_and_send_next_mission() # detect mission
    if response:
        print(flight.decode_error(response))
        return

    # wait for the drone to reach the detect zone
    flight.detect_mission.wait_for_waypoint_reached(1, 30)

    # Capture images
    camera.capture_images(30, 0.15, './test_photos') 

    # Detect the targets
    targets = lion_sight.detect_targets()

    # run an airdrop mission for each target
    for target in targets:

        # Build the airdrop mission
        response = flight.build_airdrop_mission(target, 'airdrop_mission.txt', 0, 7500, 282)
        if response:
            print(flight.decode_error(response))
            return

        # Append the airdrop mission
        flight.append_airdrop_mission()

        # wait and run next mission
        response = flight.wait_and_send_next_mission() # airdrop mission
        if response:
            print(flight.decode_error(response))
            return

    # wait and run next mission
    response = flight.wait_and_send_next_mission() # land mission

    # if the land mission fails
    if response:

        # print the error
        print(flight.decode_error(response))
        
        # wait for drone to recover
        time.sleep(30)

        # retry the land mission
        flight.land_mission.send_mission()
        flight.controller.set_mode('AUTO')



if __name__ == '__main__':
    main()