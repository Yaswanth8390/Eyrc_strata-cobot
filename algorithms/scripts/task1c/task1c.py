#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1C of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          7024
# Author List:		M YASWANTH REDDY, SHREEYANTH D
# Filename:		    task1c.py
# Functions:
#			        process_navigation is the major one which contains the logic and uses pure pursuit for navigation and potential fields for obstacle avoidance
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /cmd_vel ]
#                   Subscribing Topics - [ /ebot_path, /odom, /scan, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
##################### TASK CONSTANTS #######################

# The route is published here ONCE, before your node starts, and held for late
# subscribers. An ordinary subscription waits forever for a message already sent.
path_topic = '/ebot_path'

odom_topic = '/odom'
scan_topic = '/scan'
cmd_topic = '/cmd_vel'
map_topic = '/map'
# What /cmd_vel accepts. The base clips anything beyond these, and clipping one of the
# two changes their ratio - which is the arc the base actually drives.
max_linear_mps = 0.5
max_angular_rps = 1.0

##################### CLASS DEFINITION #######################

class ebot_nav(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the eBot along the route
                    published on /ebot_path, through its waypoints, in order.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ebot_nav
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'ebot_nav_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ############ Topic SUBSCRIPTIONS ############

        # ->  NOTE: The subscription to 'path_topic' below uses the default settings and will
        #           NEVER receive the route - see the INSTRUCTIONS in 'pathcb' and fix it there.
        self.path_sub = self.create_subscription(Path, path_topic, self.pathcb, qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomcb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scancb, qos_profile_sensor_data)
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.mapcb, qos)

        ############ Topic PUBLISHERS ############

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)                      # the eBot drives on what you publish here

        ############ Constructor VARIABLES/OBJECTS ############

        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.timer = self.create_timer(control_rate, self.process_navigation)           # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)

        self.route = None                                                               # the route to drive (from pathcb())
        self.odom = None                                                                # where the base is (from odomcb())
        self.scan = None                                                                # what the lidar sees (from scancb())
        self.yaw = None
                                                                         # the heading of the base (from odomcb())
        self.obstacles = None
        self.current_wp= 0


    def pathcb(self, data):
        '''
        Description:    Callback function for the route topic.
                        Use this function to receive the waypoints the eBot has to drive.

        Args:
            data (Path):    The route, as a sequence of poses

        Returns:
        '''
        self.route = data.poses

    def odomcb(self, data):
        '''
        Description:    Callback function for the odometry topic.
                        Use this function to receive where the base currently is.

        Args:
            data (Odometry):    Pose and velocity of the base

        Returns:
        '''
        self.odom = data.pose.pose.position,data.pose.pose.orientation
        p = self.odom[1]
        self.yaw = euler_from_quaternion([p.x,p.y,p.z,p.w])[2]


    def scancb(self, data):
        '''
        Description:    Callback function for the lidar topic.
                        Use this function to receive what the lidar currently sees.

        Args:
            data (LaserScan):    One lidar sweep

        Returns:
        '''
        self.scan = data

        ############################################

    def mapcb(self,data):
        '''
        Description:    Callback function for the map topic.
                        Use this function to receive the map of the environment.

        Args:
            data (OccupancyGrid):    The occupancy grid map
        returns:
        '''
        if self.obstacles is not None:
            return
        info = data.info
        grid = np.array(data.data, dtype=np.int8).reshape(info.height, info.width)
        rows, cols = np.nonzero(grid > 50)
        xs = info.origin.position.x + (cols + 0.5) * info.resolution
        ys = info.origin.position.y + (rows + 0.5) * info.resolution
        self.obstacles = np.column_stack((xs, ys))

    def process_navigation(self):
        '''
        Description:    Timer function used to drive the eBot along the route.

        Args:
        Returns:
        '''
        if self.route is None or self.odom is None or self.scan is None or self.obstacles is None:
            return

        x = self.odom[0].x
        y = self.odom[0].y
        pts = [(pose.pose.position.x, pose.pose.position.y) for pose in self.route]
        l_ahead = 0.5
        v = 0.3

        dist = math.hypot(pts[-1][0] - x, pts[-1][1] - y)
        if dist < 0.1:
            self.cmd_pub.publish(Twist())
            return
        
        end = len(pts)
        self.current_wp = min(range(self.current_wp,end), key=lambda i: math.hypot(pts[i][0] - x, pts[i][1] - y))
        goal = pts[-1]
        for i in range(self.current_wp, len(pts)):
            if math.hypot(pts[i][0] - x, pts[i][1] - y) >= l_ahead:
                goal = pts[i]
                break
        
        L = math.hypot(goal[0] - x, goal[1] - y)
        #repulsion only exists inside this distance
        d0 = 0.4
        k_rep = 0.01

        a = self.scan.angle_min + np.arange(len(self.scan.ranges)) * self.scan.angle_increment + self.yaw
        r = np.array(self.scan.ranges)
        ok = np.isfinite(r) & (r > self.scan.range_min) & (r < d0)
        scan_pts = np.column_stack((x + r[ok] * np.cos(a[ok]), y + r[ok] * np.sin(a[ok])))
        obs = np.vstack((self.obstacles, scan_pts))

        gh = math.atan2(goal[1] - y, goal[0] - x)
        fx, fy = math.cos(gh), math.sin(gh)

        dx = obs[:, 0] - x
        dy = obs[:, 1] - y
        d = np.hypot(dx, dy)
        d = np.where(d > 0.05, d, np.inf)
        i = np.argmin(d)
        if d[i] < d0:
            mag = min(0.4, k_rep * (1.0 / d[i] - 1.0 / d0) / d[i] ** 2)
            fx -= mag * dx[i] / d[i]
            fy -= mag * dy[i] / d[i]

        alpha = math.atan2(fy, fx) - self.yaw
        #wrapping alpha to a sensible range
        alpha = math.atan2(math.sin(alpha),math.cos(alpha))
        K = 2 * math.sin(alpha) / L

        if abs(alpha) > 1.0:
            v = 0.0
            w = max(-max_angular_rps, min(max_angular_rps, 2.0*alpha))
        else:
            if abs(v*K) > max_angular_rps:
                v = max_angular_rps / abs(K)
            w = v* K

        #v is limited to 0.3 and w is calculated using the pure pursuit algorithm
        cmd = Twist()
        cmd.linear.x = min(v, max_linear_mps)
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ebot_nav class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('ebot_nav_process')                    # creating ROS node

    node.get_logger().info('Node created: eBot navigation process') # logging information

    ebot_nav_class = ebot_nav()                                     # creating a new object for class 'ebot_nav'

    rclpy.spin(ebot_nav_class)                                      # spining on the object to make it alive in ROS 2 DDS

    ebot_nav_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()
