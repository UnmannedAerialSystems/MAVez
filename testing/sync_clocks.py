import time
from MAVez import flight_controller
from MAVez.safe_logger import configure_logging
from MAVez.coordinate import Coordinate
import asyncio

class ClockSync:

    def __init__(self):
        self.controller = flight_controller.FlightController(connection_string='tcp:127.0.0.1:5762', 
                                                    baud=57600, 
                                                    logger=logger, 
                                                    message_host='127.0.0.1', 
                                                    message_port=5555, 
                                                    message_topic='',
                                                    timesync=True)
        
        self.telemetry = asyncio.Queue()
        self.clocks = asyncio.Queue()
        self.interval_s = 1  # 0.2 seconds
        self.rolling_diff_ms = []
        self.avg_diff_ms = 0.0

    async def intermittent_clock(self, ref_monotonic_ns=None):
        interval_ns = int(self.interval_s * 1e9)

        # Set initial reference time
        if ref_monotonic_ns is None:
            ref_monotonic_ns = time.monotonic_ns()

        next_tick = ref_monotonic_ns + interval_ns

        while True:
            now = time.monotonic_ns()
            sleep_ns = next_tick - now

            if sleep_ns > 0:
                await asyncio.sleep(sleep_ns / 1e9)

            # Tick happens here
            await self.clocks.put(next_tick)
            print("Clock tick at:", next_tick)

            # Schedule next tick
            next_tick += interval_ns

    async def read_gps_time(self):

        while True:
            response = await self.controller.receive_gps(normalize_time=True)
            if not isinstance(response, Coordinate):
                logger.info(f"GPS reception failed with error code: {response}")
                return

            print("GPS Time:", response.timestamp)
            await self.telemetry.put(response.timestamp)

    async def matcher(self):
        while self.telemetry.qsize() == 0:
            await asyncio.sleep(0.1)
        while True:
            if self.telemetry.qsize() > 0:
                telem = await self.telemetry.get()
                while self.clocks.qsize() > 0:
                    clock = await self.clocks.get()
                    time_diff = abs(telem - clock)
                    if time_diff < 1e8:  # within 100 ms
                        logger.info(f"Difference: {time_diff / 1e6} ms")
                        self.rolling_diff_ms.append(telem - clock)
                        if len(self.rolling_diff_ms) > 4:
                            self.rolling_diff_ms.pop(0)
                        self.avg_diff_ms = sum(self.rolling_diff_ms) / len(self.rolling_diff_ms)
                        print("Updated average difference (ms):", self.avg_diff_ms / 1e6)
                        break
            await asyncio.sleep(0.01)

    async def run(self):
        await self.controller.start()
        await self.controller.set_message_interval(33, int(self.interval_s * 1e6))  # 1 Hz

        gps_task = asyncio.create_task(self.read_gps_time())


        clock_job = asyncio.create_task(self.intermittent_clock(await self.telemetry.get()))
        await asyncio.sleep(1)  # wait a bit to gather some GPS data

        match_task = asyncio.create_task(self.matcher())
        try:
            await asyncio.gather(gps_task, match_task, clock_job)
        except asyncio.CancelledError:
            gps_task.cancel()
            match_task.cancel()
            clock_job.cancel()
            raise


logger = configure_logging()

async def main():
    clock_sync = ClockSync()
    await clock_sync.run()


if __name__ == "__main__":
    asyncio.run(main())

