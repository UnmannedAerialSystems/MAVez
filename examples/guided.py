import asyncio

from MAVez import FlightController, Coordinate, enums
from MAVez.safe_logger import configure_logging


logger = configure_logging()
async def main():
    controller = FlightController(
        connection_string='tcp:127.0.0.1:5762', 
        baud=57600, 
        logger=logger,
        message_host="127.0.0.1",
        message_port=5555,
        message_topic="",
        timesync=True
        )
    
    async with controller:

        # await controller.set_mode("GUIDED")
        # await controller.set_message_interval(enums.MAVMessage.GLOBAL_POSITION_INT, 500_000)
        # await controller.arm()

        # await controller.takeoff(0, 20)

        # coord_1 = Coordinate(latitude_deg=-35.362139, longitude_deg=149.1638792, altitude_m=30, heading_deg=180)
        # await controller.send_reposition(coord_1, radius_m=80)
        # await controller.wait_for_position_reached(coord_1, 20, 60)

        # coord_2 = Coordinate(latitude_deg=-35.362139, longitude_deg=149.1638792, altitude_m=10, heading_deg=45).offset_coordinate(0.01, 180)
        # coord_2.altitude_m = 10
        # await controller.send_reposition(coord_2, radius_m=80, relative_yaw=True)
        # await controller.wait_for_position_reached(coord_2, 5, 60)
        await asyncio.sleep(1)
        controller.override_rc(4, 2000)
        await asyncio.sleep(2.5)
        controller.release_rc(4)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

