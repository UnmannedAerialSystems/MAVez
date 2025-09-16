from MAVez import flight_controller
from logging import Logger

logger = Logger(__name__)

controller = flight_controller.FlightController(connection_string='tcp:172.0.0.1:5762', baud=57600, logger=logger)
print("Connected to vehicle")

controller.arm()
print("Vehicle armed")

controller.takeoff("./examples/sample_missions/takeoff_mission.txt")
print("Takeoff mission started")

# controller.append_mission("./examples/sample_missions/detect_mission.txt")

# controller.append_mission("./examples/sample_missions/landing_mission.txt")

# controller.wait_and_send_next_mission()

# controller.wait_and_send_next_mission()

# controller.await_landing()

# controller.disarm()

