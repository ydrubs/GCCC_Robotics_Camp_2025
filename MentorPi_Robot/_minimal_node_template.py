#!/usr/bin/env python3
"""
Minimal ROS 2 Node Template for MentorPi

This file serves as a clean starting point for any ROS 2 script on the MentorPi.
It includes:
- Node setup
- Optional structure for publishers, subscribers, and timers
- Clean startup and shutdown handling

Customize this file by uncommenting and replacing the message types and topic names.
"""

import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')

        """
        A publisher is used to send data to other nodes. 
        You define the message type, topic name, and queue size.
        Example use: sending motor commands, LED colors, etc.
        """
        # self.publisher = self.create_publisher(YourMsgType, '/your_topic', 10)


        """
        A subscription listens to a specific topic and triggers a callback
        whenever a new message is received. Useful for reacting to sensor input,
        button presses, etc.
        """
        # self.create_subscription(YourMsgType, '/your_topic', self.your_callback, 10)


        """
        A timer lets you run a function at regular intervals (in seconds).
        Commonly used for control loops or periodic updates.
        """
        # self.create_timer(1.0, self.timer_callback)  # every 1 second


        """
        The logger provides terminal output for debugging and feedback.
        """
        self.get_logger().info('MinimalNode has started.')

    """
    Callback functions are triggered by subscriptions or timers. 
    You can define one or more to react to incoming data or periodic events.
    """


    # def your_callback(self, msg):
    """
    This is a sample callback function for a subscriber.

    It gets automatically called whenever a new message is received 
    on the topic this node is subscribed to. The `msg` parameter contains 
    the data published by the other node, and its structure depends on 
    the message type specified during subscription.

    You can use this function to process the incoming message, trigger
    robot behavior, log data, or publish a response.

    Example:
        If subscribing to a std_msgs/msg/String topic,
        `msg.data` will be the string value that was published.
    """
    #     self.get_logger().info(f'Received: {msg.data}')


    # def timer_callback(self):
    """
    This is a sample timer callback function.

    It gets called automatically at a regular interval, as defined by the timer
    created in the node's `__init__()` method using `create_timer()`.

    Timers are useful for:
    - Repeatedly publishing messages (e.g., velocity commands or sensor data)
    - Performing control loop checks
    - Logging or monitoring at a steady rate

    Example:
        If the timer is set with `self.create_timer(1.0, self.timer_callback)`,
        this function will be called once every 1.0 second.
    """
    #     self.get_logger().info('Timer triggered.')


def main(args=None):
    """
    Initializes ROS 2, creates and spins the node,
    and performs clean shutdown on exit.
    """
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
