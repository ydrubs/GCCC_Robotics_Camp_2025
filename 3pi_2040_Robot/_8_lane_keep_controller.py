"""
BoundaryAvoidanceController

This controller allows the robot to move forward while avoiding black lines,
such as tape boundaries. It is useful for teaching basic sensor calibration,
line detection, and simple autonomous navigation.

How it works:
-------------
- The robot calibrates its five line sensors by spinning in place, allowing each sensor
  to detect both black (tape) and white (floor) surfaces.
- After calibration, the robot moves forward at a steady speed.
- If one or more sensors detect a strong black reading (above a set threshold), the robot
  begins to steer away from the detected side.
- Steering continues until the black line is no longer detected.

Key Parameters:
---------------
- border_threshold: Minimum sensor value considered "black". Typically 600–900 depending on lighting.
- speed: Forward driving speed. Recommended range: 1500–3000.
- turn_strength: How sharply the robot steers to avoid a black line.
- avoiding_timeout: How long the robot keeps steering before re-evaluating sensor readings.

Calibration & Tuning:
---------------------
To get accurate black line detection, the robot needs to calibrate its sensors correctly.
During calibration, the robot should:
- Spin in place to expose all sensors to both black and white surfaces.
- Be placed so the sensors sweep over black tape and a light floor.

Students can experiment with these values:
- **border_threshold**: Try increasing this if the robot is falsely detecting dark floor as black tape. Decrease if the robot misses lines.
- **speed**: A slower speed (e.g. 1500) gives better reaction time for tight turns or narrow tape.
- **turn_strength**: If the robot turns too sharply or not sharply enough, adjust this to steer more or less aggressively.
- **avoiding_timeout**: A longer timeout (e.g. 300–500 ms) keeps the robot turning longer, while a shorter one makes it recheck more often.

Tip: Observe the sensor bar graph on the display — taller bars mean darker surfaces. Use this to fine-tune your `border_threshold`.
"""


from pololu_3pi_2040_robot import robot
import time

class BoundaryAvoidanceController:
    """
    This controller drives the robot forward while gently steering away from black lines
    (like tape borders). It uses the line sensors to detect black surfaces and adjusts motor
    speeds to avoid them.

    Calibration: Press A when the robot is in position to calibrate (white floor & black tape).
    After calibration, press A again to start driving.

    Parameters:
        - border_threshold: IR sensor reading that counts as 'black' (suggested range: 600–900)
        - speed: forward movement speed (suggested: 1500–3000)
    """

    def __init__(self):
        self.motors = robot.Motors()
        self.sensors = robot.LineSensors()
        self.display = robot.Display()
        self.button_a = robot.ButtonA()
        self.speed = 1500
        self.border_threshold = 850
        self.avoiding = False
        self.last_seen_side = None

    def calibrate(self):
        self.display.fill(0)
        self.display.text("Press A to calibrate", 0, 10)
        self.display.show()
        while not self.button_a.check():
            pass

        self.display.fill(0)
        self.display.text("Calibrating...", 0, 10)
        self.display.show()

        # Spin left
        self.motors.set_speeds(600, -600)
        for _ in range(50):
            self.sensors.calibrate()
            time.sleep_ms(20)

        # Spin right
        self.motors.set_speeds(-600, 600)
        for _ in range(100):
            self.sensors.calibrate()
            time.sleep_ms(20)

        # Spin left again
        self.motors.set_speeds(600, -600)
        for _ in range(50):
            self.sensors.calibrate()
            time.sleep_ms(20)

        self.motors.off()

        self.display.fill(0)
        self.display.text("Done. Press A", 0, 10)
        self.display.text("to start driving.", 0, 20)
        self.display.show()
        while not self.button_a.check():
            pass

    def update(self):
        sensors = self.sensors.read_calibrated()
        left = sensors[0] > self.border_threshold or sensors[1] > self.border_threshold
        right = sensors[3] > self.border_threshold or sensors[4] > self.border_threshold

        steer = 0.0
        if left and not right:
            steer = -0.4
            self.last_seen_side = "left"
        elif right and not left:
            steer = 0.4
            self.last_seen_side = "right"
        elif left and right:
            steer = -0.4 if self.last_seen_side == "left" else 0.4
        else:
            self.last_seen_side = None

        # Apply motor speeds
        left_speed = int(self.speed * (1.0 - steer))
        right_speed = int(self.speed * (1.0 + steer))
        self.motors.set_speeds(left_speed, right_speed)

        # Display sensor bars
        self.display.fill_rect(0, 0, 128, 12, 0)
        for i, value in enumerate(sensors):
            h = int(value * 24 / 1000)
            self.display.fill_rect(25 + i * 12, 64 - h, 8, h, 1)
        self.display.show()
