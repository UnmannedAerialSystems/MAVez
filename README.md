# PSU UAS MAVez

**The Pennsylvania State University**

## Description

Library for controlling ArduPilot from an external computer via pymavlink.

For detailed documentation on pymavlink, visit [mavlink.io](https://mavlink.io/en/). "Standard Messages/Commands" > "common.xml" is a particularly useful resource.

## Table of Contents

- [Installation](#installation)
- [Example Usage](#example-usage)
- [Module Overview](#module-overview)
- [Error Codes](#error-codes)
- [License](#license)
- [Authors](#authors)

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

controller.prefight_check("sample_missions/landing_mission.txt", "sample_missions/geofence.txt") # unspecified home coordinate uses current

controller.arm() # must arm before takeoff

controller.takeoff(takeoff_mission.txt) # provide takeoff mission at time of takeoff

controller.append_detect_mission("sample_missions/detect_mission.txt") # provide a detect mission

controller.wait_and_send_next_mission() # wait until takeoff completes, send detect mission
```

## LICENSE:

This project is licensed under the [GNU General Public License v3.0](LICENSE).

## Authors:

[Ted Tasman](https://github.com/tedtasman)
[Declan Emery](https://github.com/dec4234)
[Vlad Roiban](https://github.com/Vladdapenn)
