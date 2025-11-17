from MAVez import flight_controller
from MAVez.safe_logger import configure_logging
from MAVez.coordinate import Coordinate
from uas_messenger.subscriber import Subscriber
import asyncio
import matplotlib.pyplot as plt

logger = configure_logging()
async def main():
    
    offset_lock = asyncio.Lock()
    offset = 0
    async def callback(msg):
        nonlocal offset
        async with offset_lock:
            offset = msg.header.get('offset', None)
        print(f"Clock offset updated: {offset} us")

    sub = Subscriber(host='127.0.0.1', port=5555, callback=callback, topics=['mavlink_timesync_update'])
    sub.start()
    

    # Using async context manager to handle messaging setup and teardown
    async with flight_controller.FlightController(connection_string='tcp:127.0.0.1:5762', 
                                                    baud=57600, 
                                                    logger=logger, 
                                                    message_host='127.0.0.1', 
                                                    message_port=5555, 
                                                    message_topic='mavlink') as controller:
        last_telemetry = None
        times = []
        offsets = []
        jitters = []
        await controller.set_message_interval(33, 100000)  # 10 Hz

        for _ in range(50):
            response = await controller.receive_gps()
            if not isinstance(response, Coordinate):
                logger.info(f"GPS reception failed with error code: {response}")
                return
            async with offset_lock:
                if last_telemetry is not None:
                    print(f"Jitter: {abs((response.timestamp * 1e6 - offset) - last_telemetry - 1e8)} us")
                last_telemetry = response.timestamp * 1e6 - offset
                times.append(last_telemetry)
                offsets.append(offset)
                jitters.append(abs((response.timestamp * 1e6 - offset) - last_telemetry - 1e8))

        expected = [i * 1e8 for i in range(50)]
        plt.scatter(x=expected, y=times, marker='o')
        plt.xlabel('Expected Time (us)')
        plt.ylabel('Received Time (us)')
        plt.title('GPS Telemetry Reception Times')
        plt.grid(True)
        plt.show()

        plt.figure()
        plt.plot(offsets)
        plt.plot(jitters)
        plt.xlabel('Sample Index')
        plt.ylabel('Clock Offset (us)')
        plt.title('Clock Offsets Over Time')
        plt.grid(True)
        plt.show()

        plt.figure()
        plt.plot(jitters)
        plt.xlabel('Sample Index')
        plt.ylabel('Jitter (us)')
        plt.title('Jitter Over Time')
        plt.grid(True)
        plt.show()

        await sub.close()


if __name__ == "__main__":
    asyncio.run(main())

