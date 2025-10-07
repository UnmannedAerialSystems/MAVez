```rst
PSU UAS MAVez
=============
**The Pennsylvania State University**

A Python library for controlling **ArduPilot** from an external computer via **pymavlink**.

For detailed documentation on `pymavlink`, visit the official site:  
`mavlink.io <https://mavlink.io/en/>`_.  
The **"Standard Messages/Commands" → "common.xml"** section is a particularly useful reference.

Module Overview
===============

- `**Controllers** <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.controllers>`__
  - Main controller classes for interfacing with the ArduPilot.
  - `Controller <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.controller.Controller>`__ - Basic controller for atomic MAVLink communication.
  - `FlightController <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.flight_controller.FlightController>`__ - Specialized controller with methods for handling complex MAVLink interactions.
- `**Missions** <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.mission>`__ 
  - Classes for creating and managing missions.
  - `Mission <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.mission.Mission>`__ - Represents a full mission with multiple waypoints.
  - `MissionItem <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.mission_item.MissionItem>`__ - Represents a single waypoint or command in a mission.
- `**Coordinate** <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.coordinate>`__
  - Class representing geographic coordinates with latitude, longitude, and altitude.
  - Includes methods for distance and bearing calculations.
- `**General Utilities** <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.utils>`__
  - General utility functions and classes for the library.
  - `**ZeroMQ Broker** <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.zmq_broker>`__ - Implements a ZeroMQ broker for inter-process communication.
  - `**Safe Logger** <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.safe_logger>`__ - Only logs messages if a logger is provided.
  - `**Translate Message** <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.translate_message>`__ - Utility for translating MAVLink messages to human-readable formats.
  - `**Enums** <https://unmannedaerialsystems.github.io/MAVez/api.html#MAVez.enums>`__ - Enumerations for various MAVLink constants and types.


