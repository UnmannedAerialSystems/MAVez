import asyncio

from MAVez.controller import Controller
from MAVez.enums.mav_message import MAVMessage

async def main():
    async with Controller() as controller:
        await asyncio.sleep(3)
        print("active")
        while True:
            state = controller.get_state()

            old_state = controller.get_state()
  
            await asyncio.sleep(2)
            
            new_state = controller.get_state()
            
            print("Old:", old_state.attitude_timestamp)
            print("New:", new_state.attitude_timestamp)
            print()

            await asyncio.sleep(0.05)

asyncio.run(main())