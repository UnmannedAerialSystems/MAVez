# PSU UAS MAVez
## The Pennsylvania State University

## Description
A repository for controlling a UAV with MavLink for the UAS competition.

## Table of Contents
- [Module Overview](#module-overview)
- [Error Codes](#error-codes)

## Module Overveiw

### [`flight_manager.py`](#flight-manager)
Responsible for managing the flight of the vehicle.

### [`mav_controller.py`](#mav-controller)
Responsible for connecting to, communicating with, and sending commands to the drone.

## Flight Manager
*Responsible for managing the flight of the vehicle*
### Attributes:
- `controller`: [Controller](#mav-controller) object
- `takeoff_mission`: [Mission](#Mission) object to store the takeoff mission
- `land_mission`: [Mission](#Mission) object to store the landing mission
- `detect_mission`: [Mission](#Mission) object to store the detection mission
- `airdrop_mission`: [Mission](#Mission) object to store the airdrop mission
- `geofence`: [Mission](#Mission) object to store the geofence boundary
- `mission_list`: List to maintain mission queue
- `preflight_check_done`: Boolean tracker for completion of preflight checks
### Methods:
**Unless otherwise specified, all methods return 0 or falsy value on success**

`__init__`:
| Description | Inputs | Outputs |
| ----------- | ------ | ------- |
| Initializes all attributes | `connection string`: string to pass to controller | `None` |

`decode_error`:
| Description | Inputs | Outputs |
| ----------- | ------ | ------- |
| Decodes an error code to english message. Determines origin of the error and passes it to the decoder of the module it originated from. | `error_code`: Integer error code (see [Error Codes](#error-codes)) | string of error message |

`takeoff`:
| Description | Inputs | Outputs |
| ----------- | ------ | ------- |
| Handles takeoff procedure | `takeoff_mission_file`: string filepath to takeoff mission | *On failure*: error code |

`append_airdrop_mission`:
| Description | Inputs | Outputs |
| ----------- | ------ | ------- |
| Appends airdrop mission attribute to mission queue if it has been built *(see [build_airdrop_mission](#build_airdrop_mission))*| `None` | *On failure*: error code |

`append_detect_mission`:
| Description | Inputs | Outputs |
| ----------- | ------ | ------- |
| Loads detect mission from file and appends it to mission queue | `None` | *On failure*: error code |

`build_airdrop_mission`:
| Description | Inputs | Outputs |
| ----------- | ------ | ------- |
| Loads airdrop mission from file up to `target_index`. Appends 3 Mission items in a line along `heading`, `buffer_distance` apart, at `altitude`, with the middle point at `target_coordinate` |  `target_coordinate`: Coordinate object at location of airdrop target| *On failure*: error code |
| | `airdrop_mission_file`: string filepath to airdrop mission | |
| | `target_index`: index in airdrop mission file where target should be dropped | |
| | `altitude`: altitude for entry, drop, and exit points | |
| | `heading`: heading in degrees which entry and exit points will be generated along. **default value: 0**| |
| | `buffer_distance`: gap in meters between entry and exit points and target point. **default value: 100** | |

`append_mission`:
| Description | Inputs | Outputs |
| ----------- | ------ | ------- |
| Appends any mission file to mission queue. | `filename`: string filepath to mission | *On failure*: error code |

`wait_for_waypoint_reached`:
| Description | Inputs | Outputs |
| ----------- | ------ | ------- |
| Waits for a specific waypoint sequence number to be reached. **Blocking**. | `target`: integer sequence number to wait for. | *On failure*: error code |
| | `timeout`: maximum waiting time *between waypoints* in seconds. **default value: 10** 

`wait_and_send_next_mission`:
| Description | Inputs | Outputs |
| ----------- | ------ | ------- |
| Waits for end of current mission to be reached, then sends next mission in mission queue. **Blocking**. | `None`  | *On failure*: error code |

`wait_for_landed`:
| Description | Inputs | Outputs |
| ----------- | ------ | ------- |
| Waits to receive message indicating vehicle has landed. **Blocking**. | `timeout`: how long to wait for landing to occur in seconds. **default value: 30** | *On failure*: error code |

`preflight_check`:
| Description | Inputs | Outputs |
| ----------- | ------ | ------- |
| Sets home coordinate, sets geofence, prepares landing mission, enables geofence, sets `preflight_check_done` to `True`. | `land_mission_file`: string filepath to landing mission | *On failure*: error code |
| | `geofence_file`: string filepath to geofence "mission" | |
| | `home_coordinate`: Coordinate object where vehicle home will be set. **default value: *use current location*** | |

## MAV Controller
*Responsible for connecting to, communicating with, and sending commands to the drone.*
### Methods:
`await_mission_item_reached`
## Mission

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

