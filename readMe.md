# PSU UAS MAVez
**The Pennsylvania State University**

## Description
Library for controlling ArduPilot from an external computer via pymavlink.

For detailed documentation on pymavlink, visit [mavlink.io](https://mavlink.io/en/). "Standard Messages/Commands" > "common.xml" is a particulary useful resource.

## Table of Contents
- [Installation](#installation)
- [Example Usage](#example-usage)
- [Module Overview](#module-overview)
- [Error Codes](#error-codes)

## Installation
1. In a terminal window, run `git clone git@github.com:UnmannedAerialSystems/MAVez.git`
2. Switch into the newly cloned directory by running `cd MAVez`
3. Install the required dependencies by running `pip install -r requirements.txt`
4. Create a python file in the parent directory of MAVez
```
your_project/
  ├── your_python_script.py
  └── MAVez/
```
5. At the top of your file, import your desired modules with `from MAVez import Coordinate, flight_manager, ...`

While not required, it is highly recommended that you set up [ArduPilot's Software in the Loop (SITL)](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) simulator to make testing significantly easier.

## Example Usage
Below is a simple script designed to work with SITL, assuming the directory structure is as described in the installation.
```Python
from MAVez import flight_manager

controller = flight_manager.Flight(connection_string='tcp:127.0.0.1:5762') # connection string for SITL

controller.prefight_check(landing_mission.txt, geofence.txt) # unspecified home coordinate uses current

controller.arm() # must arm before takeoff

controller.takeoff(takeoff_mission.txt) # provide takeoff mission at time of takeoff

controller.append_detect_mission(detect_mission.txt) # provide a detect mission

controller.wait_and_send_next_mission() # wait until takeoff completes, send detect mission
```

## Module Overview
- [`Coordinate.py`](#coordinate)
  - Representation of a GPS coordinate, functionality for comparing between coordinates
- [`Mission_Item.py`](#Mission-Item)
  - Representation of a singular MAVLink [MISSION_ITEM_INT](https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT)
- [`Mission.py`](#Mission)
  - Representation of a complete ArduPilot mission
- [`mav_controller.py`](#mav-controller)
  - Lowest level of abstraction for communicating with Ardupilot via pymavlink, atomic methods
- [`flight_manager.py`](#flight-manager)
  - Extension of the controller class, with multiple operations combined into singular methods

## Coordinate
*A GPS coordinate compatible with MAVLink*
- Attributes:
  - `is_int`: Boolean flag indicating whether latitude/longitude are represented in degrees or integer microdegrees (x10^7). Integer coordinates are required for many MAVlink messages
  - `lat`: Latitude of the coordinate (unit depends on `is_int)
  - `lon`: Longitude of the coordinate (unit depends on `is_int)
  - `alt`: Altitude of the coordinate. Always in float meters
- Methods:
  - `dms_to_dd`: Convert lat or lon from degrees,minutes,seconds to decimal degrees
  - `offset_coordinate`: Find a new coordinate a specified distance away along a specified heading
  - `normalize`: Converts another coordinate object to utilize the same `is_int` status
  - `distance_to`: Distance from `self` to another coordinate object
  - `bearing_to`: Compass bearing from `self` to another coordiante object

## Mission Item
*A waypoint; a singular [MISSION_ITEM_INT](https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT)*
- Attributes:
  - `seq`: index/position of this waypoint in the mission
  - `frame`: the [MAV_FRAME](https://mavlink.io/en/messages/common.html#MAV_FRAME) of the coordinate
  - `command`: the [MAV_CMD](https://mavlink.io/en/messages/common.html#mav_commands) for this waypoint
  - `current`: boolean indicator whether this is the current waypoint or not
  - `auto_continue`: boolean indicator whether the mission should automatically proceed after reaching this waypoint
  - `x`: integer latitude of the coordinate
  - `y`: integer longitude of the coordinate
  - `z` = altitude of the coordinate
  - `param1`: first parameter
  - `param2`: second parameter
  - `param3`: third parameter
  - `param4`: fourth parameter
  - `type`: the [MAV_MISSION_TYPE](https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE) of this mission.
- Properties
  - `Message`: returns the MAVLink message for this mission item
 
## Mission
*A list of Mission Items making up a waypoint mission or geofence*
- Attributes:
  - `controller`: a Controller or Flight object for ArduPilot communication
  - `type`: the [MAV_MISSION_TYPE](https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE) of this mission.
  - `mission_items`: a list of mission item objects
- Methods:
  - `load_mission_from_file`: loads a mission from a file in the QGC WPL 110 format. (see https://mavlink.io/en/file_formats/).
  - `save_mission_to_file`: saves the mission to a file in the QGC WPL 110 format. (see https://mavlink.io/en/file_formats/).
  - `send_mission`: sends the mission to ArduPilot using `self.controller`.
  - `clear_mission`: clears the currently loaded mission in ArduPilot using `self.controller`

## MAV Controller
*Responsible for direct communication with ArduPilot via atomic messages*
- Attributes:
  - `logger`: an object for logging activity. **Optional**
  - `master`: the master MAVLink connection to ArduPilot
- Methods (sending):
  - `send_message`: sends a generic message to ArduPilot via master connection.
  - `send_mission_count`: sends a [MISSION_COUNT](https://mavlink.io/en/messages/common.html#MISSION_COUNT) message via master connection.
  - `send_clear_mission`: sends a [MISSION_CLEAR_ALL](https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL) message via master connection.
  - `set_mode`: sets the vehicle mode with [MAV_COMMAND_DO_SET_MODE](https://mavlink.io/en/messages/common.html#MAV_COMMAND_DO_SET_MODE).
  - `arm`: attempts to arm the vehicle with [MAV_CMD_COMPONENT_ARM_DISARM](https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM).
  - `disarm`: disarms the vehicle with [MAV_CMD_COMPONENT_ARM_DISARM](https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM).
  - `enable_geofence`: activates the already sent geofence with [MAV_CMD_DO_FENCE_ENABLE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FENCE_ENABLE).
  - `disable_geofence`: deactivates geofence with [MAV_CMD_DO_FENCE_ENABLE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FENCE_ENABLE).
  - `set_home`: sets the home coordinate with [MAV_CMD_DO_SET_HOME](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME).
  - `set_servo`: sets a specified servo to a specified PWM with [MAV_CMD_DO_SET_SERVO](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO).
  - `set_message_interval`: sets the interval for a specified message to be sent from ArduPilot with [MAV_COMMAND_SET_MESSAGE_INTERVAL](https://mavlink.io/en/messages/common.html#MAV_COMMAND_SET_MESSAGE_INTERVAL).
  - `disable_message_interval`: sets the a specified message to stop transmitting from ArduPilot with [MAV_COMMAND_SET_MESSAGE_INTERVAL](https://mavlink.io/en/messages/common.html#MAV_COMMAND_SET_MESSAGE_INTERVAL).
  - `await_current_mission_index`: waits to recieve a [MISSION_CURRENT](https://mavlink.io/en/messages/common.html#MISSION_REQUEST) message from ArduPilot.
  - `set_current_mission_index`: set the index for the next waypoint within a mission with [MAV_COMMAND_DO_SET_MISSION_CURRENT](https://mavlink.io/en/messages/common.html#MAV_COMMAND_DO_SET_MISSION_CURRENT).
  - `start_mission`: starts the mission with [MAV_COMMAND_MISSION_START](https://mavlink.io/en/messages/common.html#MAV_COMMAND_MISSION_START).
- Methods (recieving):
  - `await_mission_request`: waits to recieve a [MISSION_REQUEST](https://mavlink.io/en/messages/common.html#MISSION_REQUEST) message from ArduPilot.
  - `await_mission_ack`: waits to recieve a [MISSION_ACK](https://mavlink.io/en/messages/common.html#MISSION_ACK) message from ArduPilot.
  - `await_mission_item_reached`: waits to recieve a [MISSION_ITEM_REACHED](https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED) message from ArduPilot.
  - `recieve_channel_input`: waits to recieve an [RC_CHANNELS](https://mavlink.io/en/messages/common.html#RC_CHANNELS) message via master connection.
  - `recieve_gps`: waits to recieve a [GLOBAL_POSITION_INT](https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT) message via master connection.
  - `recieve_landing_status`: waits to recieve an [EXTENDED_SYS_STATE](https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE) message via master connection.

## Flight Manager
*Responsible for managing the flight of the vehicle. Extends the controller class.*
- Attributes
  - `logger`: an object for logging activity. **Optional**
  - `master`: the master MAVLink connection to ArduPilot
  - `takeoff_mission`: [Mission](#Mission) object to store the takeoff mission
  - `land_mission`: [Mission](#Mission) object to store the landing mission
  - `detect_mission`: [Mission](#Mission) object to store the detection mission
  - `airdrop_mission`: [Mission](#Mission) object to store the airdrop mission
  - `geofence`: [Mission](#Mission) object to store the geofence boundary
  - `mission_list`: List to maintain mission queue
  - `preflight_check_done`: Boolean tracker for completion of preflight checks
- Methods:
  - `takeoff`: Handles takeoff procedure
  - `append_mission`: Appends a mission file to mission queue.
  - `wait_for_waypoint_reached`: Waits for a specific waypoint sequence number to be reached. **Blocking**.
  - `wait_and_send_next_mission`: Waits for end of current mission to be reached, then sends next mission in mission queue. **Blocking**.
  - `wait_for_landed`: Waits to receive message indicating vehicle has landed. **Blocking**. 
  - `preflight_check`: Sets home coordinate, sets geofence, prepares landing mission, enables geofence, sets `preflight_check_done` to `True`. 
  - `wait_for_channel_input`: Waits for a specified RC channel to reach a specified PWM value. **BLOCKING**
  - `get_altitude`: Gets the current altitude of the vehicle.


## Error Codes
**Code Distribution**:
- `100-199`: Controller errors
- `200-299`: Mission errors
- `300-399`: Flight errors

| Code | Name | Description |
|------|------|-------------|
| `101` | TIMEOUT ERROR | Timed out waiting for a response from the drone.  |
| `111` | UNKNOWN MODE | The requested mode does not exist. |
| `201` | FILE NOT FOUND |No file found with given filename/filepath. |
| `202` | FILE EMPTY |The given mission file is empty |
| `203` | START OUT OF RANGE | The given start index is out of range of the mission file |
| `204` | END OUT OF RANGE |The given end index is out of range of the mission file |
| `301` | PREFLIGHT CHECK ERROR |Takeoff was attempted before preflight check was completed |
| `302` | DETECT LOAD ERROR | Attempted to append detect mission without providing file to load or loading it first |
| `303` | AIRDROP NOT BUILT ERROR | Attempted to append or build airdrop mission without providing file to load or loading it first |

## Authors:
Original Creator: (Ted Tasman)[`https://github.com/tedtasman`]
