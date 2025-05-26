#!/usr/bin/env python3
"""
Button-Controlled LED Feedback for MentorPi

This ROS 2 node listens for button presses on the topic
/ros_robot_controller/button and updates the RGB LED color accordingly.

Functionality:
- When Button 1 is pressed (id = 1, state = 1), LED index 1 is set to red.
- When Button 2 is pressed (id = 2, state = 1), LED index 1 is turned off.

The node publishes color updates to the topic /ros_robot_controller/set_rgb
using the RGBStates message format. This allows the robot to visually respond
to button inputs for simple feedback or debugging.

LED index 1 is assumed to be the primary user-facing indicator.
"""

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import ButtonState, RGBStates, RGBState

class ButtonRGBController(Node):

    def __init__(self):
        super().__init__('button_rgb_controller')

        # Subscribe to the button state topic
        self.create_subscription(
            ButtonState,
            '/ros_robot_controller/button',
            self.button_callback,
            10)

        # Publisher for RGB LED state
        self.publisher = self.create_publisher(
            RGBStates,
            '/ros_robot_controller/set_rgb',
            10)

    def button_callback(self, msg):
        # Button 1 pressed --> turn LED RED
        if msg.id == 1 and msg.state == 1:
            self.get_logger().info('Button 1 pressed! Turning LED index 1 RED.')

            rgb_msg = RGBStates()
            rgb_state = RGBState()
            rgb_state.index = 1    # LED index
            rgb_state.red = 255
            rgb_state.green = 0
            rgb_state.blue = 0

            rgb_msg.states.append(rgb_state)
            self.publisher.publish(rgb_msg)

        # Button 2 pressed --> turn LED OFF
        elif msg.id == 2 and msg.state == 1:
            self.get_logger().info('Button 2 pressed! Turning LED index 1 OFF.')

            rgb_msg = RGBStates()
            rgb_state = RGBState()
            rgb_state.index = 1    # LED index
            rgb_state.red = 0
            rgb_state.green = 0
            rgb_state.blue = 0

            rgb_msg.states.append(rgb_state)
            self.publisher.publish(rgb_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ButtonRGBController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
