from _8_lane_keep_controller import BoundaryAvoidanceController
import time

controller = BoundaryAvoidanceController()
controller.calibrate()

while True:
    controller.update()
    time.sleep(0.01)
