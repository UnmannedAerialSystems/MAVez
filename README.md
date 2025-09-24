# PSU UAS MAVez

**The Pennsylvania State University**

## Description

Library for controlling ArduPilot from an external computer via pymavlink.

For detailed documentation on pymavlink, visit [mavlink.io](https://mavlink.io/en/). "Standard Messages/Commands" > "common.xml" is a particularly useful resource.

## Table of Contents

- [Installation](#installation)
- [Example Usage](#example-usage)
- [License](#license)
- [Authors](#authors)
- [Appendix](#appendix)

## Installation

1. In a terminal window, run `git clone git@github.com:UnmannedAerialSystems/MAVez.git`
2. Switch into the newly cloned directory by running `cd MAVez`
3. Install the package locally in editable mode `pip install -e .`

While not required, it is highly recommended that you utilize [ArduPilot's Software in the Loop (SITL)](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) simulator to make testing significantly easier. [Install instructions](#sitl-installation) are provided for Windows and MacOS in the appendix. For Linux or WSL instructions, refer to the [ArduPilot setup page](https://ardupilot.org/dev/docs/SITL-setup-landingpage.html)

## Example Usage

Below is a simple script designed to work with SITL. This example is provided at `/examples/basic.py/`

```Python
from MAVez import flight_controller
from MAVez.safe_logger import configure_logging
import asyncio


logger = configure_logging()
async def main():
    controller = flight_controller.FlightController(connection_string='tcp:127.0.0.1:5762', baud=57600, logger=logger)

    await controller.set_geofence("./examples/sample_missions/sample_fence.txt")

    await controller.arm()

    await controller.takeoff("./examples/sample_missions/sample1.txt")
    controller.append_mission("./examples/sample_missions/sample2.txt")
    controller.append_mission("./examples/sample_missions/sample3.txt")

    await controller.auto_send_next_mission()
    await controller.auto_send_next_mission()

    await controller.wait_for_landing()

    await controller.disarm()

if __name__ == "__main__":
    asyncio.run(main())
```

## License:

This project is licensed under the [GNU General Public License v3.0](LICENSE).

## Authors:

[Ted Tasman](https://github.com/tedtasman)
[Declan Emery](https://github.com/dec4234)
[Vlad Roiban](https://github.com/Vladdapenn)

## Appendix:

### SITL Installation

#### Windows

SITL comes precompiled for Windows as part of Mission Planner. [Installing Mission Planner](https://ardupilot.org/planner/docs/mission-planner-installation.html) is the most painless method for using SITL on Windows.

#### MacOS

1. Install prerequisites:

```bash
brew install python3 gcc
pip install future pymavlink MAVProxy opencv-python
```

2. Clone ArduPilot repository

```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

3. Install environment

```bash
cd Tools/environment_install
./install-prereqs-mac.sh  # decline all prompts
```

4. Compile the binaries

Return to ./ardupilot

```bash
./waf configure --board sitl
./waf copter
./waf plane
```

5. Run SITL

```bash
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```
