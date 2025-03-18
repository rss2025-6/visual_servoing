#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class LineFollower(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("line_follower")

        self.declare_parameter("drive_topic", "/vesc/low_level/input/navigation")
        self.DRIVE_TOPIC = self.get_parameter("drive_topic").get_parameter_value().string_value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)
        
        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        self.parking_distance = 0.02 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #drive_cmd.header.frame_id = "base_link"
        #drive_cmd.header.stamp = self.get_clock().now().to_msg()

        #################################
        
        # YOUR CODE HERE

        # set constant speed for line following
        drive_cmd.drive.speed = 0.7

        # set steering angle based on distance & side of the line
        drive_cmd.drive.steering_angle = self.relative_y
        

        #################################
        
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.distance_error=np.sqrt(self.relative_x**2+self.relative_y**2)-self.parking_distance
        error_msg.x_error=self.relative_x-self.parking_distance
        error_msg.y_error=self.relative_y

        #################################
        
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    lf = LineFollower()
    rclpy.spin(lf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
