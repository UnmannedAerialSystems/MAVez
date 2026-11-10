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
            age = controller.get_message_age(MAVMessage.ATTITUDE)
            stale = controller.is_stale(MAVMessage.ATTITUDE, 0.5)

            print(f"Attitude age: {age:.3f}s")
            print(f"Is stale (0.5s threshold): {stale}")

            await asyncio.sleep(0.05)

asyncio.run(main())