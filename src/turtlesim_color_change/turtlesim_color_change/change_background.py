#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from rclpy.qos import QoSProfile
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

class ChangeBackgroundColor(Node):
    def __init__(self):
        super().__init__('change_background_color')
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            QoSProfile(depth=10)
        )
        self.set_parameters_client = self.create_client(SetParameters, '/turtlesim/set_parameters')
        while not self.set_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetParameters.Request()
        self.future = None
    def send_request(self, r, g, b):
        self.req.parameters = [
            Parameter(name='background_r', value=r).to_parameter_msg(),
            Parameter(name='background_g', value=g).to_parameter_msg(),
            Parameter(name='background_b', value=b).to_parameter_msg()
        ]
        self.future = self.set_parameters_client.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
    def pose_callback(self, msg):
        if msg.x < 1.0:
            # Hit the left wall
            print("Hit the left wall")
            self.send_request(0,0,255)
            print("Color changed to blue")
        elif msg.x > 10.0:
            # Hit the right wall
            print("Hit the right wall")
            self.send_request(0,255,0)
            print("Color changed to green")
        elif msg.y > 10.0:
            # Hit the top wall
            print("Hit the top wall")
            self.send_request(255,0,0)
            print("Color changed to red")
        elif msg.y < 1.0:
            # Hit the bottom wall
            print("Hit the bottom wall")
            self.send_request(255,165,0)
            print("Color changed to orange")
def main(args=None):
    rclpy.init(args=args)
    node = ChangeBackgroundColor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

