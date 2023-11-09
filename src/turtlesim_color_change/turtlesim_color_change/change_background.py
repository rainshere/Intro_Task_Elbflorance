#!/usr/bin/env python

# Import necessary libraries and modules
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose # Pose message type
from rclpy.qos import QoSProfile
from rcl_interfaces.srv import SetParameters # SetParameters service
from rclpy.parameter import Parameter

# Define a class for the node that changes the background color
class ChangeBackgroundColor(Node):
    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('change_background_color')
        # Create a subcription to the '/turtlesim/pose' topic with a callback function
        self.subscription = self.create_subscription(
            Pose, 
            'turtle1/pose',
            self.pose_callback,
            QoSProfile(depth=10)
        )
        # Create a client to the '/turtlesim/set_parameters' service
        self.set_parameters_client = self.create_client(SetParameters, '/turtlesim/set_parameters')
        
        # Wait for the service to be available
        while not self.set_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object for the SetParameters service
        self.req = SetParameters.Request()
        self.future = None

    # Method to send a request to change the background color
    def send_request(self, r, g, b):
        self.req.parameters = [
            Parameter(name='background_r', value=r).to_parameter_msg(),
            Parameter(name='background_g', value=g).to_parameter_msg(),
            Parameter(name='background_b', value=b).to_parameter_msg()
        ]
        # Call the SetParameters service asynchronously
        self.future = self.set_parameters_client.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)

    # Callback function for the Pose subcription 
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

# Main Function to initialize and run the node
def main(args=None):
    rclpy.init(args=args)
    node = ChangeBackgroundColor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

