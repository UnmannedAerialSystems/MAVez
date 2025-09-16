from MAVez import flight_controller

controller = flight_controller.FlightController(connection_string='tcp:127.0.0.1:5762')

controller.arm()

controller.takeoff("sample_missions/takeoff.txt")

controller.append_mission("sample_missions/detect_mission.txt")

controller.append_mission("sample_missions/land.txt")

controller.wait_and_send_next_mission()

controller.wait_and_send_next_mission()

controller.await_landing()

controller.disarm()

