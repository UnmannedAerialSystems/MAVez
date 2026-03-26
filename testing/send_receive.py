from pathlib import Path

from MAVez import Controller, safe_logger, Mission, MAVMessage
import logging
import asyncio

async def main():
    logger = safe_logger.configure_logging(logging.INFO)
    con = Controller(
        logger=logger,
        timesync=True
    )
    await con.start()
    test_mission = Mission.from_file(
        controller=con,
        filepath=Path("./examples/sample_missions/sample1.txt")
    )
    await con.arm()
    await con.set_mode("AUTO")
    await test_mission.send_mission() if test_mission else print("oops")
    await asyncio.sleep(0.5)
    await con.set_message_interval(MAVMessage.WIND, int(0.5*1e6))
    while True:
        data = await con.receive_wind(normalize_direction=True)
        print(data)


if __name__ == "__main__":
    asyncio.run(main())