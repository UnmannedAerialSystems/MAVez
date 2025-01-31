# PSU UAS MavEZ
## The Pennsylvania State University

## Description
A repository for controlling a UAV with MavLink for the UAS competition.

## Table of Contents
- [Modules](#modules)
- [Classes](#classes)
- [Error Codes](#error-codes)

## Modules

### `flight_manager.py`
**Current version: 1.0.0**
Responsible for managing the flight of the drone.

### `mav_controller.py`
**Current version: 1.0.0**
Responsible for connecting to, communicating with, and sending commands to the drone.

### `flight_utils.py`
**Current version: 1.0.0**
Contains utility classes for managing missions and waypoints.

### `target_mapper.py`
**Current version: 1.0.0**
Converts object detection results into coordinates.
- Currently a placeholder to allow functionality of other modules.

## Classes

### `flight_manager.py`
- **Flight**: Manages drone flight plan

### `mav_controller.py`
- **Controller**: Facilitates communication with the drone.

### `flight_utils.py`
- **Coordinate**: Represents a coordinate in latitude, longitude, and altitude.
- **Mission_Item**: Represents a mission item (waypoint) for the drone.
- **Mission**: Represents a mission plan for the drone.

### `target_mapper.py`
- **Target_Mapper**: Converts object detection results into coordinates.

## Error Codes
**Code Distribution**:
- `100-199`: Controller errors
- `200-299`: Mission errors

| Code | Description |
|------|-------------|
| `101` | Timed out waiting for a response from the drone.  |
| `111` | The requested mode does not exist. |
| `201` | No file found with given filename/filepath. |
| `202` | The given mission file is empty |
| `203` | The given start index is out of range of the mission file |
| `204` | The given end index is out of range of the mission file |
