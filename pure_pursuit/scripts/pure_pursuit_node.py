#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import os
from math import *
from time import sleep
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# TODO CHECK: include needed ROS msg type headers and libraries
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        self.odom_sub = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.pose_callback,
            10)

        self.odom_sub  # prevent unused variable warning

        self.drive_pub = self.create_publisher( \
            AckermannDriveStamped, '/drive', 10)

        self.marker_pub = self.create_publisher(Marker, 'vis', 10)
        
        self.cwd = '/sim_ws/src/pure_pursuit/scripts/'
        wps = np.genfromtxt(self.cwd+'levine_blocked_waypoints.csv', delimiter = ',')
        uniques = np.unique(wps, axis = 0, return_index = True)
        idxs = np.sort(uniques[1])
        self.unique_wps = wps[idxs]
        self.unique_xys = self.unique_wps[:,:2]
        self.l = 1.
        self.odom_x = 0
        self.odom_y = 0
        self.odom_theta = 0
        self.id = 0
    
    def draw_marker(self, x, y, id, c):

        if c == 'b':
            c_arr = [0., 0., 1.]
        elif c== 'r':
            c_arr = [1., 0., 0.]

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.id = id
        marker.type = Marker.SPHERE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.scale.x = .1
        marker.scale.y = .1
        marker.scale.z = .1
        marker.color.r = c_arr[0]
        marker.color.g = c_arr[1]
        marker.color.b = c_arr[2]
        marker.color.a = 1.
        marker.lifetime.sec = 1000
        
        self.marker_pub.publish(marker)

    def get_yaw(self, pose_msg):

        for [x,y] in self.unique_xys:
            self.draw_marker(x, y, self.id, 'r')
            self.id += 1

        quaternion = np.array([pose_msg.pose.pose.orientation.x, 
                        pose_msg.pose.pose.orientation.y, 
                        pose_msg.pose.pose.orientation.z, 
                        pose_msg.pose.pose.orientation.w])

        euler = euler_from_quaternion(quaternion)

        return euler[2]

    def pose_callback(self, pose_msg):
        #('l', self.l)

        # define vars
        self.odom_x = pose_msg.pose.pose.position.x
        self.odom_y = pose_msg.pose.pose.position.y
        self.odom_theta = self.get_yaw(pose_msg)

        #print('odom', self.odom_x, self.odom_y, self.odom_theta)

        # TODO: find the current waypoint to track using methods mentioned in lecture
        odom_l_x = self.odom_x + self.l*cos(self.odom_theta)
        odom_l_y = self.odom_y + self.l*sin(self.odom_theta)
        #print('odom_l_x & y', odom_l_x, odom_l_y)

        # calculate euclidian distance of each wp to the cars current position + lookahead distance
        euc_dists = np.sqrt(np.sum((self.unique_xys - \
            [odom_l_x, odom_l_y]) ** 2, axis = 1))
        
        # find argmin of euc_dists
        min_dist_idx = np.argmin(euc_dists)

        # find wp closest to odom_l_x and odom_l_y and record it
        wp_x = self.unique_xys[min_dist_idx][0]
        wp_y = self.unique_xys[min_dist_idx][1]

        #print('curr wp', wp_x, wp_y)

        self.draw_marker(wp_x, wp_y, self.id, 'b')
        self.id += 1

        # TODO: transform goal point to vehicle frame of reference
        delta_x = wp_x - self.odom_x
        delta_y = wp_y - self.odom_y

        #print('delta x y', delta_x, delta_y)

        transformed_x = delta_x*cos(self.odom_theta)+delta_y*sin(self.odom_theta)
        transformed_y = -delta_x*sin(self.odom_theta)+delta_y*cos(self.odom_theta)

        #print('transformed x y', transformed_x, transformed_y)

        # TODO: calculate curvature/steering angle
        steering_angle = 2*transformed_y / (np.hypot(transformed_x, transformed_y) ** 2)
        steering_angle = 0.6*steering_angle
        #print('pre steering angle', steering_angle)

        if steering_angle >= 30*pi/180:
            steering_angle = 30*pi/180
        elif steering_angle <= -30*pi/180:
            steering_angle = -30*pi/180

        #print('steering_angle', steering_angle)

        # TODO: publish drive message, don't forget to limit the steering angle.
        drive = AckermannDrive(speed = 0.4, \
            steering_angle = steering_angle)
        drive_msg = AckermannDriveStamped(drive = drive)
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
