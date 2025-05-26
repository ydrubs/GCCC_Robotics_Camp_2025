"""
Battery LED Status Indicator for MentorPi

This ROS 2 node subscribes to the robot's battery voltage topic
(/ros_robot_controller/battery) and monitors the current battery level.

If the voltage is above 7.5V (7500 mV), it turns LED 1 green to indicate
a healthy battery. If the voltage is 7.5V or lower, it turns LED 1 red
to indicate that charging is recommended soon.

The node also logs the current voltage to the terminal using the ROS logger.
"""

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from ros_robot_controller_msgs.msg import RGBStates, RGBState
from std_msgs.msg import UInt16


class BatteryStatus(Node):
    def __init__(self):
        super().__init__('battery_status_led')

        self.create_subscription(
            UInt16,
            '/ros_robot_controller/battery',
            self.battery_callback,
            10
        )

        self.publisher = self.create_publisher(
            RGBStates,
            '/ros_robot_controller/set_rgb',
            10)

    def battery_callback(self, msg):
        volts = msg.data
        if msg.data > 7500:
            self.get_logger().info(f"Battery is good, {volts}")

            rgb_msg = RGBStates()
            rgb_state = RGBState()
            rgb_state.index = 1  # LED index
            rgb_state.red = 0
            rgb_state.green = 255
            rgb_state.blue = 0

            rgb_msg.states.append(rgb_state)
            self.publisher.publish(rgb_msg)

        else:
            self.get_logger().info(f"Battery needs charging soon")
            rgb_msg = RGBStates()
            rgb_state = RGBState()
            rgb_state.index = 1  # LED index
            rgb_state.red = 255
            rgb_state.green = 0
            rgb_state.blue = 0

            rgb_msg.states.append(rgb_state)
            self.publisher.publish(rgb_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryStatus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()