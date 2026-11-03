import asyncio

from MAVez.controller import Controller
from MAVez.enums.mav_message import MAVMessage

async def main():
    async with Controller() as controller:
        await asyncio.sleep(3)
        print("active")
        while True:
            state = controller.get_state()

            print("\n--- Vehicle State ---")
            print("Position:", state.position)
            print("Velocity:", state.velocity)
            print("Attitude:", state.attitude)
            print("Angular Velocity:", state.angular_velocity)
            print("Global Position:", state.global_position)
            print("Timestamp:", state.timestamp)

            await asyncio.sleep(0.5)

asyncio.run(main())