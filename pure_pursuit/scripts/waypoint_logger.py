#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import csv

# TODO CHECK: include needed ROS msg type headers and libraries
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nav_msgs.msg import Odometry

import numpy as np

class WaypointLogger(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('waypoint_logger')
        # TODO: create ROS subscribers and publishers
        self.odom_sub = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.listener_callback,
            10)

        self.odom_sub  # prevent unused variable warning

        self.file = open('/test_1.csv', 'w')
        self.writer = csv.writer(self.file)

    def listener_callback(self, msg):  

        quaternion = np.array([msg.pose.pose.orientation.x, 
                        msg.pose.pose.orientation.y, 
                        msg.pose.pose.orientation.z, 
                        msg.pose.pose.orientation.w])


        euler = euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([msg.twist.twist.linear.x, 
                                msg.twist.twist.linear.y, 
                                msg.twist.twist.linear.z]),2)
        final_arr = [msg.pose.pose.position.x, \
            msg.pose.pose.position.y, euler[2], speed]
        
        print(final_arr)
        self.writer.writerow(final_arr)

    
def main(args=None):
    rclpy.init(args=args)
    print("Waypoint Logger Initialized")
    waypoint_logger_node = WaypointLogger()
    rclpy.spin(waypoint_logger_node)
    waypoint_logger_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
