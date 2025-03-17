#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic", "/vesc/low_level/input/navigation")
        self.DRIVE_TOPIC = self.get_parameter("drive_topic").get_parameter_value().string_value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)
        
        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        self.parking_distance = 0.02 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.prev_time = self.get_clock().now().to_msg().nanosec
        self.KP = 0.9 #2.5
        self.KD = 2.0
        self.last_error = 0
        self.get_logger().info(f"Parking Controller Initialized KP = {self.KP}")
        self.get_logger().info(self.DRIVE_TOPIC)

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #drive_cmd.header.frame_id = "base_link"
        #drive_cmd.header.stamp = self.get_clock().now().to_msg()

        #################################
        
        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd
        current_error = self.relative_x - self.parking_distance
        #angle = 3.14 - np.arctan2(self.relative_y, self.relative_x)
        angle = self.relative_y
        
        dt = self.get_clock().now().to_msg().nanosec - self.prev_time
        self.prev_time = self.get_clock().now().to_msg().nanosec

        # calculate our control signal based on Kp * error + Kd * d/dt(error)
        distance_control_signal = self.KP*current_error + self.KD*(current_error/dt)
        drive_cmd.drive.speed = 0.7
        #max_speed = 0.99
        #min_speed = 0.7
        #if distance_control_signal > 0:
        #    drive_cmd.drive.speed = max(min_speed, min(distance_control_signal, max_speed))
        #else:
        #    drive_cmd.drive.speed = min(-min_speed, max(distance_control_signal, -max_speed))

        #if current_error > 0:
        #     drive_cmd.drive.speed = angle
        #else:
        #     drive_cmd.drive.speed = -angle

        #if current_error < 0.1 and current_error > -0.1:
        #    drive_cmd.drive.speed = 0.


        self.last_error = current_error
        
        #if current_error < 0:
        #    drive_cmd.drive.steering_angle = -angle
        #else:
        #    drive_cmd.drive.steering_angle = angle
        self.get_logger().info(f"ANGLE = {angle:.2f}")
        drive_cmd.drive.steering_angle = angle
        

        #################################
        #self.get_logger().info(f"ERROR = {current_error:.2f}")
        
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
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
