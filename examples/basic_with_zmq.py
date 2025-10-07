"""
A simple ZMQ subscriber example that connects to a ZMQ the controller's built-in broker and listens for all messages.

To run this example, first start the controller with ZMQ broker enabled:
    python examples/basic_with_zmq.py
Then run this subscriber script:
    python examples/zmq_subscriber.py
"""

from MAVez import flight_controller
from MAVez.safe_logger import configure_logging
import asyncio


logger = configure_logging()
async def main():
    controller = flight_controller.FlightController(connection_string='tcp:127.0.0.1:5762', 
                                                    baud=57600, 
                                                    logger=logger, 
                                                    zmq_host='localhost', 
                                                    zmq_port=5555, 
                                                    zmq_topic='mavlink')

    await controller.set_geofence("./examples/sample_missions/sample_fence.txt")

    await controller.arm()

    await controller.takeoff("./examples/sample_missions/sample1.txt")
    controller.append_mission("./examples/sample_missions/sample2.txt")
    controller.append_mission("./examples/sample_missions/sample3.txt")

    await controller.auto_send_next_mission()
    await controller.auto_send_next_mission()

    await controller.wait_for_landing()

    await controller.disarm()



if __name__ == "__main__":
    asyncio.run(main())

