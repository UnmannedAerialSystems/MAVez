import asyncio

from MAVez.controller import Controller
from MAVez.enums.mav_message import MAVMessage

async def main():
    async with Controller() as controller:
        await asyncio.sleep(3)

        while True:
            gps = controller.get_latest_message(
                MAVMessage.GLOBAL_POSITION_INT
            )

            attitude = controller.get_latest_message(
                MAVMessage.ATTITUDE
            )

            print(f"GPS: {gps}")
            print(f"ATTITUDE: {attitude}")
            print()

            await asyncio.sleep(0.5)

asyncio.run(main())