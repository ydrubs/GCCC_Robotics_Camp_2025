"""
This code is necessary for the wind spring challenge code to function correctly
"""

from pololu_3pi_2040_robot import robot
import time

class LineCrossController:
    """
    LineCrossController

    This controller handles:
    - Calibrating the line sensors by spinning in place.
    - Detecting when a line is fully crossed (edge detection).
    - Playing a buzzer sound on each crossing.
    - Filtering out repeated triggers while still over the line.

    Use .crossed_line() to check if a new line has been crossed.
    """

    def __init__(self, threshold=800):
        self.sensors = robot.LineSensors()
        self.buzzer = robot.Buzzer()
        self.display = robot.Display()
        self.motors = robot.Motors()
        self.button_a = robot.ButtonA()
        self.threshold = threshold
        self.last_seen = False

    def calibrate(self, spin_speed=1500):
        """
        Spins in place and calibrates the sensors. Waits for button A.
        """
        self.display.fill(0)
        self.display.text("Place on tape &", 0, 10)
        self.display.text("white. Press A", 0, 20)
        self.display.show()

        while not self.button_a.check():
            pass

        self.display.fill(0)
        self.display.text("Calibrating...", 0, 10)
        self.display.show()

        # Spin left
        self.motors.set_speeds(spin_speed, -spin_speed)
        for _ in range(50):
            self.sensors.calibrate()
            time.sleep(0.02)

        # Spin right
        self.motors.set_speeds(-spin_speed, spin_speed)
        for _ in range(100):
            self.sensors.calibrate()
            time.sleep(0.02)

        # Spin left again
        self.motors.set_speeds(spin_speed, -spin_speed)
        for _ in range(50):
            self.sensors.calibrate()
            time.sleep(0.02)

        self.motors.off()

        self.display.fill(0)
        self.display.text("Done. Press A", 0, 10)
        self.display.show()

        while not self.button_a.check():
            pass

    def crossed_line(self):
        """
        Returns True only once when a line is crossed by the center sensor.
        Plays a buzzer sound, moves forward slightly to clear the line,
        and filters out repeated triggers.
        """
        center = self.sensors.read_calibrated()[2]
        is_line = center > self.threshold

        if is_line and not self.last_seen:
            self.last_seen = True
            self.buzzer.play("g4")

            # Move forward briefly to fully cross the line
            self.motors.set_speeds(1200, 1200)
            # time.sleep(0.2)
            self.motors.off()

            return True
        elif not is_line:
            self.last_seen = False

        return False

# === Optional test code (commented out) ===
# if __name__ == "__main__":
#     controller = LineCrossController()
#     controller.calibrate()
#     motors = robot.Motors()
#     while True:
#         if controller.crossed_line():
#             print("Line crossed!")
#         motors.set_speeds(1500, 1500)
#         time.sleep(0.01)
