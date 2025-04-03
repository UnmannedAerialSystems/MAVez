# PSU UAS MAVez
## The Pennsylvania State University

## Description
A repository for controlling a UAV with MavLink for the UAS competition.

## Table of Contents
- [Modules](#modules)
- [Classes](#classes)
- [Error Codes](#error-codes)

## Modules

### [`flight_manager.py`](#flight-manager)
Responsible for managing the flight of the drone.

### `mav_controller.py`
Responsible for connecting to, communicating with, and sending commands to the drone.

## Flight Manager
*Responsible for 

## Error Codes
**Code Distribution**:
- `100-199`: Controller errors
- `200-299`: Mission errors
- `300-399`: Flight errors

| Code | Description |
|------|-------------|
| `101` | Timed out waiting for a response from the drone.  |
| `111` | The requested mode does not exist. |
| `201` | No file found with given filename/filepath. |
| `202` | The given mission file is empty |
| `203` | The given start index is out of range of the mission file |
| `204` | The given end index is out of range of the mission file |
| `301` | Takeoff was attempted before preflight check was completed |
| `302` | Attempted to append detect mission without providing file to load or loading it first |

