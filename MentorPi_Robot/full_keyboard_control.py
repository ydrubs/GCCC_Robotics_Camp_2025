"""
Keyboard Teleoperation for MentorPi Ackermann Robot

This script allows manual control of an Ackermann-steered robot using keyboard input
within a terminal (via curses). The robot uses ROS 2 topics to control drive speed
and steering angle. It publishes linear velocity commands to `/controller/cmd_vel`
and steering PWM signals to `/ros_robot_controller/pwm_servo/set_state`.

Key Commands:
    q     : Quit the program
   up     : Drive forward with centered wheels
 down     : Drive backward with centered wheels
    d     : Steer left and drive forward
    a     : Steer right and drive forward
    z     : Steer left and drive backward
    c     : Steer right and drive backward
    -     : Decrease speed by 0.1 (min: 0.1)
    +/=   : Increase speed by 0.1 (max: 0.8)

Notes:
- Speed adjustments affect all drive commands (forward and reverse).
- Servo ID is assumed to be 3.
- This script must be run in a terminal with curses support.
- Before running the script:
    1. Stop app control:
       ~/.stop_ros.sh
    2. Launch the ROS controller:
       ros2 launch controller controller.launch.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import SetPWMServoState, PWMServoState
import curses
import time


class KeyDrive(Node):
    def __init__(self, stdscr):
        super().__init__('key_drive_servo')
        self.vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.servo_pub = self.create_publisher(SetPWMServoState, '/ros_robot_controller/pwm_servo/set_state', 10)

        self.servo_id = 3
        self.center = 1500
        self.left = 2000
        self.right = 1000

        self.current_drive = 0.0
        self.current_steering = self.center
        self.last_key_time = 0
        self.active_cmd = None

        self.speed = 0.2  # default speed
        self.min_speed = 0.1
        self.max_speed = 0.8

        self.running = True
        self.stdscr = stdscr
        self.stdscr.nodelay(True)

        self.run_loop()

    def send_drive(self, speed):
        if speed != self.current_drive:
            msg = Twist()
            msg.linear.x = speed
            self.vel_pub.publish(msg)
            self.current_drive = speed
            self.get_logger().info(f"[Drive] {speed:.2f}")

    def send_servo(self, pos):
        if pos != self.current_steering:
            msg = SetPWMServoState()
            state = PWMServoState()
            state.id = [self.servo_id]
            state.position = [pos]
            state.offset = [0]
            msg.state.append(state)
            self.servo_pub.publish(msg)
            self.current_steering = pos
            self.get_logger().info(f"[Steer] {pos}")

    def adjust_speed(self, delta):
        old_speed = self.speed
        self.speed = max(self.min_speed, min(self.max_speed, self.speed + delta))
        if self.speed != old_speed:
            self.get_logger().info(f"[Speed] Updated to {self.speed:.2f}")

    def run_loop(self):
        self.stdscr.addstr(0, 0, "A/D=turn fwd  Z/C=turn back  ?/?=straight  +/-=speed  Q=quit")

        while self.running:
            key = self.stdscr.getch()
            now = time.time()

            if key == ord('q'):
                break

            # Adjust speed
            elif key == ord('-'):
                self.adjust_speed(-0.1)
            elif key == ord('+') or key == ord('='):  # Some keyboards use '=' for '+'
                self.adjust_speed(0.1)

            # Forward turning
            elif key == ord('d'):
                self.send_drive(self.speed)
                self.send_servo(self.left)
                self.active_cmd = 'd'
                self.last_key_time = now

            elif key == ord('a'):
                self.send_drive(self.speed)
                self.send_servo(self.right)
                self.active_cmd = 'a'
                self.last_key_time = now

            # Reverse turning
            elif key == ord('z'):
                self.send_drive(-self.speed)
                self.send_servo(self.left)
                self.active_cmd = 'z'
                self.last_key_time = now

            elif key == ord('c'):
                self.send_drive(-self.speed)
                self.send_servo(self.right)
                self.active_cmd = 'c'
                self.last_key_time = now

            # Straight driving
            elif key == curses.KEY_UP:
                self.send_drive(self.speed)
                self.send_servo(self.center)
                self.active_cmd = 'up'
                self.last_key_time = now

            elif key == curses.KEY_DOWN:
                self.send_drive(-self.speed)
                self.send_servo(self.center)
                self.active_cmd = 'down'
                self.last_key_time = now

            # Stop and center after delay
            elif self.active_cmd and (now - self.last_key_time > 0.25):
                self.send_drive(0.0)
                self.send_servo(self.center)
                self.active_cmd = None

            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        self.send_drive(0.0)
        self.send_servo(self.center)
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    curses.wrapper(KeyDrive)


if __name__ == '__main__':
    main()
