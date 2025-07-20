Package Overview
================

- `coordinate.py`_  
  - Representation of a GPS coordinate, functionality for comparing between coordinates
- `mission_item.py`_  
  - Representation of a singular MAVLink `MISSION_ITEM_INT <https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT>`__
- `mission.py`_  
  - Representation of a complete ArduPilot mission
- `controller.py`_  
  - Lowest level of abstraction for communicating with Ardupilot via pymavlink, atomic methods
- `flight_controller.py`_
  - Extension of the controller class, with multiple operations combined into singular methods


Coordinate
----------

*A GPS coordinate compatible with MAVLink*

**Attributes:**

- ``is_int``: Boolean flag indicating whether latitude/longitude are represented in degrees or integer microdegrees (x10^7). Integer coordinates are required for many MAVlink messages
- ``lat``: Latitude of the coordinate (unit depends on ``is_int``)
- ``lon``: Longitude of the coordinate (unit depends on ``is_int``)
- ``alt``: Altitude of the coordinate. Always in float meters

**Methods:**

- ``dms_to_dd``: Convert lat or lon from degrees, minutes, seconds to decimal degrees
- ``offset_coordinate``: Find a new coordinate a specified distance away along a specified heading
- ``normalize``: Converts another coordinate object to utilize the same ``is_int`` status
- ``distance_to``: Distance from ``self`` to another coordinate object
- ``bearing_to``: Compass bearing from ``self`` to another coordinate object


Mission Item
------------

*A waypoint; a singular `MISSION_ITEM_INT <https://mavlink.io/en/messages/common.html#MISSION_ITEM_INT>`__*

**Attributes:**

- ``seq``: Index/position of this waypoint in the mission
- ``frame``: The `MAV_FRAME <https://mavlink.io/en/messages/common.html#MAV_FRAME>`__
- ``command``: The `MAV_CMD <https://mavlink.io/en/messages/common.html#mav_commands>`__ for this waypoint
- ``current``: Boolean indicator whether this is the current waypoint
- ``auto_continue``: Boolean indicator whether the mission should automatically proceed
- ``x``: Integer latitude of the coordinate
- ``y``: Integer longitude of the coordinate
- ``z``: Altitude of the coordinate
- ``param1`` - ``param4``: Mission parameters 1 through 4
- ``type``: The `MAV_MISSION_TYPE <https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE>`__ of this mission

**Properties:**

- ``Message``: Returns the MAVLink message for this mission item


Mission
-------

*A list of Mission Items making up a waypoint mission or geofence*

**Attributes:**

- ``controller``: A Controller or Flight object for ArduPilot communication
- ``type``: The `MAV_MISSION_TYPE <https://mavlink.io/en/messages/common.html#MAV_MISSION_TYPE>`__ of this mission
- ``mission_items``: A list of mission item objects

**Methods:**

- ``load_mission_from_file``: Loads a mission from a file in the QGC WPL 110 format (`reference <https://mavlink.io/en/file_formats/>`__)
- ``save_mission_to_file``: Saves the mission to a file in the QGC WPL 110 format
- ``send_mission``: Sends the mission to ArduPilot using ``self.controller``
- ``clear_mission``: Clears the currently loaded mission in ArduPilot using ``self.controller``


Controller
----------

*Responsible for direct communication with ArduPilot via atomic messages*

**Attributes:**

- ``logger``: An object for logging activity. **Optional**
- ``master``: The master MAVLink connection to ArduPilot

**Methods (sending):**

- ``send_message``
- ``send_mission_count``: Sends `MISSION_COUNT <https://mavlink.io/en/messages/common.html#MISSION_COUNT>`__
- ``send_clear_mission``: Sends `MISSION_CLEAR_ALL <https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL>`__
- ``set_mode``: Sets vehicle mode via `MAV_COMMAND_DO_SET_MODE <https://mavlink.io/en/messages/common.html#MAV_COMMAND_DO_SET_MODE>`__
- ``arm`` / ``disarm``: Arms/disarms with `MAV_CMD_COMPONENT_ARM_DISARM <https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM>`__
- ``enable_geofence`` / ``disable_geofence``: Use `MAV_CMD_DO_FENCE_ENABLE <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_FENCE_ENABLE>`__
- ``set_home``: Set home with `MAV_CMD_DO_SET_HOME <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME>`__
- ``set_servo``: Set servo PWM via `MAV_CMD_DO_SET_SERVO <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO>`__
- ``set_message_interval`` / ``disable_message_interval``: Use `MAV_COMMAND_SET_MESSAGE_INTERVAL <https://mavlink.io/en/messages/common.html#MAV_COMMAND_SET_MESSAGE_INTERVAL>`__
- ``await_current_mission_index``: Wait for `MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MISSION_CURRENT>`__
- ``set_current_mission_index``: Set waypoint index using `MAV_COMMAND_DO_SET_MISSION_CURRENT <https://mavlink.io/en/messages/common.html#MAV_COMMAND_DO_SET_MISSION_CURRENT>`__
- ``start_mission``: Start mission using `MAV_COMMAND_MISSION_START <https://mavlink.io/en/messages/common.html#MAV_COMMAND_MISSION_START>`__

**Methods (receiving):**

- ``await_mission_request``: Waits for `MISSION_REQUEST <https://mavlink.io/en/messages/common.html#MISSION_REQUEST>`__
- ``await_mission_ack``: Waits for `MISSION_ACK <https://mavlink.io/en/messages/common.html#MISSION_ACK>`__
- ``await_mission_item_reached``: Waits for `MISSION_ITEM_REACHED <https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED>`__
- ``recieve_channel_input``: Receives `RC_CHANNELS <https://mavlink.io/en/messages/common.html#RC_CHANNELS>`__
- ``recieve_gps``: Receives `GLOBAL_POSITION_INT <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>`__
- ``recieve_landing_status``: Receives `EXTENDED_SYS_STATE <https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE>`__


Flight Controller
-----------------

*Responsible for managing the flight of the vehicle. Extends the controller class.*

**Attributes:**

- ``logger``: An object for logging activity. **Optional**
- ``master``: The master MAVLink connection to ArduPilot
- ``takeoff_mission`` - ``airdrop_mission``: `Mission`_ objects for various phases
- ``geofence``: `Mission`_ object to store the geofence boundary
- ``mission_list``: List to maintain mission queue
- ``preflight_check_done``: Boolean tracker for completion of preflight checks

**Methods:**

- ``takeoff``: Handles takeoff procedure
- ``append_mission``: Appends a mission file to mission queue
- ``wait_for_waypoint_reached``: Blocks until waypoint is reached
- ``wait_and_send_next_mission``: Blocks until mission end, then sends next
- ``wait_for_landed``: Blocks until vehicle is landed
- ``preflight_check``: Prepares vehicle for flight, sets geofence, marks preflight done
- ``wait_for_channel_input``: Waits for RC channel input. **Blocking**
- ``get_altitude``: Gets the current altitude of the vehicle


Error Codes
-----------

**Code Distribution**:

- ``100-199``: Controller errors
- ``200-299``: Mission errors
- ``300-399``: Flight errors

+------+--------------------------+---------------------------------------------------------+
| Code | Name                     | Description                                             |
+======+==========================+=========================================================+
| 101  | TIMEOUT ERROR            | Timed out waiting for a response from the drone         |
| 111  | UNKNOWN MODE             | The requested mode does not exist                       |
+------+--------------------------+---------------------------------------------------------+
| 201  | FILE NOT FOUND           | No file found with given filename/filepath              |
| 202  | FILE EMPTY               | The given mission file is empty                         |
| 203  | START OUT OF RANGE       | Start index is out of mission file range                |
| 204  | END OUT OF RANGE         | End index is out of mission file range                  |
+------+--------------------------+---------------------------------------------------------+
| 301  | PREFLIGHT CHECK ERROR    | Takeoff attempted before preflight check was completed  |
+------+--------------------------+---------------------------------------------------------+


.. _coordinate.py: #coordinate
.. _mission_item.py: #mission-item
.. _mission.py: #mission
.. _controller.py: #controller
.. _flight_controller.py: #flight-controller

