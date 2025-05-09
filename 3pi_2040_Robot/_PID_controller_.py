from pololu_3pi_2040_robot import robot
import time

# === Hardware Initialization (module-wide) ===
motors = robot.Motors()
encoders = robot.Encoders()
imu = robot.IMU()
display = robot.Display()
yellow_led = robot.YellowLED()
button_c = robot.ButtonC()
button_a = robot.ButtonA()

imu.reset()
imu.enable_default()

interrupted = False  # Global interrupt flag


# === TurnController Class ===
class TurnController:
    def __init__(self, base_speed=2000, kp=280, kd=6):
        self.motors = motors
        self.encoders = encoders
        self.imu = imu
        self.base_speed = base_speed
        self.kp = kp
        self.kd = kd
        self.reset()

    def reset(self):
        self.robot_angle = 0.0
        self.target_angle = 0.0
        self.turn_rate = 0.0
        self.last_time = None
        self.drive_active = False
        self.target_ticks = None
        self.encoder_counts = [0, 0]
        self.turning = False

    def drive_straight(self, ticks):
        self.reset()
        self.encoders.get_counts(reset=True)
        self.encoder_counts = [0, 0]
        self.motors.set_speeds(800, 800)
        yellow_led.value(1)
        time.sleep(0.3)
        self.robot_angle = 0.0
        self.target_angle = 0.0
        self.last_time = time.ticks_us()
        self.target_ticks = ticks
        self.drive_active = True
        self.turning = False

    def turn_degrees(self, degrees):
        self.reset()
        self.robot_angle = 0.0
        self.target_angle = degrees
        self.last_time = time.ticks_us()
        self.drive_active = True
        self.turning = True
        yellow_led.value(1)

    def stop(self):
        self.motors.off()
        self.drive_active = False
        self.turning = False
        yellow_led.value(0)

    def update(self):
        if not self.drive_active:
            return

        if self.imu.gyro.data_ready():
            self.imu.gyro.read()
            self.turn_rate = self.imu.gyro.last_reading_dps[2]
            now = time.ticks_us()
            if self.last_time:
                dt = time.ticks_diff(now, self.last_time)
                self.robot_angle += self.turn_rate * dt / 1_000_000
            self.last_time = now

        if self.turning:
            angle_error = self.target_angle - self.robot_angle
            turn_speed = angle_error * self.kp + self.turn_rate * self.kd
            max_turn_speed = 3200  # experiment with 3000–5000
            turn_speed = max(min(int(turn_speed), max_turn_speed), -max_turn_speed)
            self.motors.set_speeds(-turn_speed, turn_speed)
            if abs(angle_error) < 1.0 and abs(self.turn_rate) < 5:
                self.stop()
        else:
            angle_error = self.robot_angle - self.target_angle
            correction = angle_error * self.kp + self.turn_rate * self.kd
            left = self.base_speed - correction
            right = self.base_speed + correction
            left = max(min(int(left), 6000), -6000)
            right = max(min(int(right), 6000), -6000)
            self.motors.set_speeds(right, left)
            self.encoder_counts = self.encoders.get_counts()
            c = self.encoder_counts
            avg_ticks = abs((c[0] + c[1]) // 2)
            if avg_ticks >= self.target_ticks:
                self.stop()

    def handle_interrupt(self):
        # from pololu_3pi_2040_robot import robot  # local import in case of isolated module use
        display = robot.Display()
        display.fill(0)
        display.text("A=start  C=stop", 0, 0, 1)
        display.show()


# === Helper: Wait Until Done ===
def wait_until_done(controller):
    global interrupted
    interrupted = False
    while controller.drive_active:
        if button_c.check() and button_c.is_pressed():
            interrupted = True
            controller.stop()
            display.fill(0)
            display.text("INTERRUPTED", 0, 0, 1)
            display.show()
            break

        controller.update()
        c = controller.encoder_counts
        display.fill_rect(0, 16, 128, 48, 0)
        display.text(f"Target:  {controller.target_angle:+.1f}", 0, 16, 1)
        display.text(f"Current: {controller.robot_angle:+.1f}", 0, 32, 1)
        display.text(f"L:{c[0]} R:{c[1]}", 0, 48, 1)
        display.show()


# === Demo/Test Mode ===
if __name__ == '__main__':

    # === Optional: Run Sequence for Testing ===
    def run_sequence():
        global interrupted
        interrupted = False

        controller.drive_straight(1000)
        wait_until_done(controller)
        if interrupted: return

        controller.turn_degrees(90)
        wait_until_done(controller)
        if interrupted: return

        controller.drive_straight(1000)
        wait_until_done(controller)
        if interrupted: return

        controller.turn_degrees(-45)
        wait_until_done(controller)


    controller = TurnController()

    display.fill(0)
    display.text("A=start  C=stop", 0, 0, 1)
    display.show()

    while True:
        if button_a.check() and not controller.drive_active:
            while button_a.check(): pass
            display.fill(0)
            display.text("A=start  C=stop", 0, 0, 1)
            display.show()
            run_sequence()

        if not controller.drive_active:
            controller.update()
            c = controller.encoder_counts
            display.fill_rect(0, 16, 128, 48, 0)
            display.text(f"Target:  {controller.target_angle:+.1f}", 0, 16, 1)
            display.text(f"Current: {controller.robot_angle:+.1f}", 0, 32, 1)
            display.text(f"L:{c[0]} R:{c[1]}", 0, 48, 1)
            display.show()
