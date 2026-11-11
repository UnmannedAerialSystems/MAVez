import asyncio
import time

from MAVez.controller import Controller
from MAVez.enums.mav_message import MAVMessage

async def main():
    async with Controller(timesync=False) as controller:
        await asyncio.sleep(3)
        print("active")
        success = await controller.test_single_timesync()

        print("TIMESYNC success:", success)

        state = controller.get_state()

        estimated_boot_seconds = (
            controller._Controller__clock_synchronizer.to_autopilot_time(
                time.monotonic_ns()
            ) / 1e9
        )

        print("Estimated boot time:", estimated_boot_seconds)
        print("Position timestamp:", state.position_timestamp)
        print("Attitude timestamp:", state.attitude_timestamp)

asyncio.run(main())