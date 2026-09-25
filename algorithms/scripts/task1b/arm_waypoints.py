#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1B of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		    task1b_boilerplate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /delta_twist_cmds, /delta_joint_cmds ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states, /etc... ]


################### IMPORT MODULES #######################
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
import sys
import math
from rclpy.node import Node
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32


##################### TASK CONSTANTS #######################

# Tool positions in base_link, in metres, in the order they must be reached. The tool
# stops at each one and holds it for at least two seconds. Copy the signs as they are:
# base_link is the UR7e's own frame, not the Gazebo world's.
waypoints = [
    (-0.4085, -0.5379, 0.1967),   # 1
    (-0.8000, -0.0005, 0.3967),   # 2
    (-0.7430,  0.5280, 0.1967),   # 3
    (-0.4097,  0.5280, 0.1967),   # 4
    (-0.0763,  0.5280, 0.1967),   # 5
]

# The two command interfaces. Only ONE is active at a time; messages to the other are
# accepted and ignored.
servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'

# The only frame a twist may be stamped with; any other is refused, not converted. Note
# 'base' is base_link turned through 180 degrees, not another name for it.
base_frame = 'base_link'

# JointJog velocities are matched to these names, in this order.
joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# What the servo accepts. A command above one of these is dropped WHOLE, not clamped.
cap_linear_mps = 0.15     # magnitude of a twist's linear part
cap_angular_rps = 0.35    # magnitude of its angular part
cap_joint_rps = 0.35      # per joint

# Dead-man switch: the arm stops this long after the last message it received.
command_timeout_s = 0.15

def cap(v, limit):
    norm = math.sqrt(sum(x**2 for x in v))
    if norm > limit:
        v = [x * (limit / norm) for x in v]
    return v

##################### CLASS DEFINITION #######################

class arm_waypoints(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the UR7e's tool through the
                    given waypoints using the arm's velocity command interfaces.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class arm_waypoints
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'arm_waypoints_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic PUBLISHERS ############

        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)    # end-effector velocity, in base_link
        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)        # per-joint velocity

        ############ Topic SUBSCRIPTIONS ############

        self.tcp_sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.tcpposecb, 20)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.jointstatecb, 50)
        self.status_sub = self.create_subscription(Int32, '/arm_status', self.armstatuscb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.switch_cli = self.create_client(                                           # client used to pick which command topic is live
            SwitchController, f'{servo_ns}/switch_controller')
        self.timer = self.create_timer(control_rate, self.process_waypoints)            # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)

        self.tcp_pose = None                                                            # tool pose variable (from tcpposecb())
        self.joint_angles = None                                                        # joint feedback variable (from jointstatecb())
        self.arm_status = None                                                          # arm state code variable (from armstatuscb())
        self.active_controller = None
        self.current_wp = 0
        self.arrival_time = None
        self.cruise_speed = 0.10
        self.phase = 'unfold'  
        self.elbow_target = None
        self.switch_controller(twist_controller)
        self.base_home = [
            0.0,
            -0.5235989992238703,
            -2.44346100157454,
            -0.17453300287162465,
            1.5707959999999992,
            1.5707959998303753
        ]

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Add any variable your motion needs to keep between cycles.
        #       ->  HINT: Which waypoint you are on, and what the arm is doing about it.

        ############################################


    def tcpposecb(self, data):
        '''
        Description:    Callback function for the tool pose topic.
                        Use this function to receive where the tool currently is.

        Args:
            data (PoseStamped):    Pose of the tool, reported in base_link

        Returns:
        '''
        self.tcp_pose = data.pose.position,data.pose.orientation
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Store the tool's position and orientation.
        #       ->  HINT: data.pose.position, data.pose.orientation

        ############################################


    def jointstatecb(self, data):
        '''
        Description:    Callback function for the joint states topic.
                        Use this function to receive the current angle of each joint.

        Args:
            data (JointState):    Joint feedback published by the arm

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :
        self.joint_angles = [data.position[data.name.index(j)] for j in joint_names]

        #	->  Store the joint angles, matched BY NAME - the order is not promised.
        #       ->  HINT: angles = [data.position[data.name.index(j)] for j in joint_names]
        #       ->  NOTE: A joint assumed to be at zero when it is not reads as a large
        #                 error, and a controller acting on it drives hard towards it.

        ############################################


    def armstatuscb(self, data):
        '''
        Description:    Callback function for the arm status topic.
                        Use this function to receive the arm's current state code.

        Args:
            data (Int32):    One state code describing what the arm is doing

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        self.arm_status = data.data
        # INSTRUCTIONS & HELP :

        #	->  Store the code, and log it while you are developing. Zero is healthy; anything
        #       else is the arm telling you about the last command or its own state.
        #       ->  HINT: /arm_status_detail says the same thing in words.
        #       ->  NOTE: A protective stop LATCHES and cannot be cleared - the run is over.

        ############################################


    def switch_controller(self, controller):
        '''
        Description:    Function to make one of the arm's two command interfaces the active
                        one, so that commands published to it are acted on.

        Args:
            controller  (str):      Name of the controller to activate, either
                                    'twist_controller' or 'joint_controller'

        Returns:
            success     (bool):     Whether the controller was activated
        '''

        ############ ADD YOUR CODE HERE ############
        req = SwitchController.Request()

        if controller == twist_controller:
            self.active_controller = twist_controller
            req.activate_controllers = [twist_controller]
            req.deactivate_controllers = [joint_controller]
        elif controller == joint_controller:
            self.active_controller = joint_controller
            req.activate_controllers = [joint_controller]
            req.deactivate_controllers = [twist_controller]

        req.strictness = SwitchController.Request.STRICT
        self.switch_cli.wait_for_service(timeout_sec=20.0)
        future = self.switch_cli.call_async(req)
        if future.done():
            response = future.result()
        else:
            return

        # INSTRUCTIONS & HELP :

        #	->  Call the switch_controller service on self.switch_cli-
        #           req = SwitchController.Request()
        #           req.activate_controllers = [ the one you want ]
        #           req.deactivate_controllers = [ the other one ]
        #           req.strictness = SwitchController.Request.STRICT

        #   ->  Wait for it first, and expect the first attempt to fail-
        #           self.switch_cli.wait_for_service(timeout_sec=20.0)
        #           future = self.switch_cli.call_async(req)
        #           rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        #       ->  NOTE: That last call HANGS if made from a callback of a node that is
        #                 already spinning. __init__ is safe; from the timer, poll
        #                 'future.done()' instead.

        #   ->  Do not switch more often than you need to.

        ############################################

    
    def process_waypoints(self):
        '''
        Description:    Timer function used to drive the tool through the waypoints.

        Args:
        Returns:
        '''
        if self.tcp_pose is None or self.joint_angles is None or self.arm_status is None:
            return
        if self.current_wp >= len(waypoints):
            return 

        if self.phase == 'unfold':
            if self.elbow_target is None:
                target_deg = -(90 + 45 * self.current_wp) 
                self.elbow_target = self.base_home[0] + math.radians(target_deg)
                self.switch_controller(joint_controller)

            raw_error = self.elbow_target - self.joint_angles[0]
            base_error = math.atan2(
                math.sin(raw_error),
                math.cos(raw_error)
            )
            if abs(base_error) < math.radians(2):
                self.phase = 'cartesian'
                self.elbow_target = None
                self.switch_controller(twist_controller)
                return
            vel = max(-cap_joint_rps, min(cap_joint_rps, base_error))
            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = joint_names

            msg.velocities = [vel, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.joint_pub.publish(msg)
            return


        if self.phase == 'return_home':
            if self.active_controller != joint_controller:
                self.switch_controller(joint_controller)

            home_error = np.array(self.base_home) - np.array(self.joint_angles)

            if np.max(np.abs(home_error)) < math.radians(2):
                if self.current_wp >= len(waypoints):
                    self.phase = 'done'
                else:
                    self.phase = 'unfold'
                self.current_wp += 1
                return


            velocities = np.clip(
                home_error,
                -cap_joint_rps,
                cap_joint_rps
            )

            msg = JointJog()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = joint_names
            msg.velocities = velocities.tolist()

            self.joint_pub.publish(msg)
            return



        ############ ADD YOUR CODE HERE ############
        current_pos,current_orien = self.tcp_pose
        kp = 1.0
        target = waypoints[self.current_wp]
        error = np.array(target) - np.array([current_pos.x, current_pos.y, current_pos.z])
        distance = np.linalg.norm(error)
        # print("Distance:",distance)
        
        cruise_speed = self.cruise_speed
        if self.arm_status != 0:
            cruise_speed *= 0.3
        if distance < 0.05:
            if self.arrival_time is None:
                self.arrival_time = self.get_clock().now()
            elif (self.get_clock().now() - self.arrival_time).nanoseconds > 2e9:
                
                self.arrival_time = None
                self.phase = 'return_home'
            return
        
        else:
            self.arrival_time = None
            direction = error / distance
            speed = min(cruise_speed, kp*distance)
            
            b = np.array([0.0, 0.0, -1.0])
            a = Rotation.from_quat([current_orien.x,current_orien.y,current_orien.z,current_orien.w]).as_matrix()[:,2]
            c = np.cross(a,b)
            norm_c = np.linalg.norm(c)
            if norm_c < 1e-8:
                omega = np.zeros(3)
            else:
                e = c / norm_c * np.arctan2(norm_c, np.dot(a, b))
                omega = kp * e
            omega = cap(omega, cap_angular_rps * 0.95)
            capped_v = cap((speed * direction).tolist(), cap_linear_mps)
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = base_frame
            msg.twist.linear.x = capped_v[0]
            msg.twist.linear.y = capped_v[1]
            msg.twist.linear.z = capped_v[2]
            msg.twist.angular.x = omega[0]
            msg.twist.angular.y = omega[1]
            msg.twist.angular.z = omega[2]
            print("Publishing:",speed * direction)
            self.twist_pub.publish(msg)
                    

        # INSTRUCTIONS & HELP :

        #	->  Return early until pose, joints and status have all arrived.

        #   ->  Both topics carry VELOCITIES, never positions.

        #   ->  Build the message for whichever interface you made active:
        #           TwistStamped on self.twist_pub  - linear and angular velocity, in 'base_frame'
        #           JointJog on self.joint_pub      - 'joint_names' and a velocity for each
        #       ->  HINT: msg.header.stamp = self.get_clock().now().to_msg()      (both)
        #                 msg.header.frame_id = base_frame                        (twist)
        #                 msg.twist.linear.x/.y/.z, msg.twist.angular.x/.y/.z     (twist)
        #                 msg.joint_names = joint_names, msg.velocities = [...]   (jog)

        #   ->  Publish on EVERY tick, zero included - see 'command_timeout_s'. Never sleep
        #       inside this function.

        #   ->  Drive the tool at a velocity proportional to the error-
        #           v = Kp * (target - current)
        #       ->  HINT: Cap it by scaling the WHOLE vector: v = v * (cap / |v|)
        #       ->  NOTE: The servo ramps down rather than stopping dead, so the arm settles
        #                 PAST the pose that satisfied you.

        #   ->  Command the tool's ORIENTATION too, or the servo decides the wrist for you.
        #       ->  HINT: The turn from unit vector 'a' onto unit vector 'b', as an axis
        #                 times an angle-
        #                     c     = cross(a, b)
        #                     e     = c / |c| * atan2(|c|, dot(a, b))
        #                     omega = Kp * e
        #                 'a' is the tool's own axis - a column of
        #                 Rotation.from_quat([x, y, z, w]).as_matrix().

        #   ->  Think about the PATH. The arm has joint limits and configurations it cannot
        #       pass through.

        #   ->  Track which waypoint you are on, and log the distance to it while developing.

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    arm_waypoints class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('arm_waypoints_process')               # creating ROS node

    node.get_logger().info('Node created: Arm waypoints process')   # logging information

    arm_waypoints_class = arm_waypoints()                           # creating a new object for class 'arm_waypoints'

    rclpy.spin(arm_waypoints_class)                                 # spining on the object to make it alive in ROS 2 DDS

    arm_waypoints_class.destroy_node()                              # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()
