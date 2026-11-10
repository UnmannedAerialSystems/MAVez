import asyncio

from MAVez.controller import Controller
from MAVez.enums.mav_message import MAVMessage

async def main():
    async with Controller() as controller:
        await asyncio.sleep(3)
        print("active")
        while True:
            state = controller.get_state()

            print("\n--- Vehicle State Timestamps ---")
            print("Position:", state.position_timestamp)
            print("Attitude:", state.attitude_timestamp)
            print("Global Position:", state.global_position_timestamp)
            print()

            await asyncio.sleep(0.05)

asyncio.run(main())