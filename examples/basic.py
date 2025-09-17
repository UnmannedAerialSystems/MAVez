from MAVez import flight_controller
from MAVez.safe_logger import configure_logging

logger = configure_logging()

controller = flight_controller.FlightController(connection_string='tcp:127.0.0.1:5762', baud=57600, logger=logger)

controller.set_geofence("./examples/sample_missions/sample_fence.txt")

controller.arm()

controller.takeoff("./examples/sample_missions/sample1.txt")

controller.append_mission("./examples/sample_missions/sample2.txt")

controller.append_mission("./examples/sample_missions/sample3.txt")

controller.wait_and_send_next_mission()

controller.wait_and_send_next_mission()

controller.await_landing()

controller.disarm()

