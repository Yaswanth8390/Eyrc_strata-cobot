#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1A of StrataCobot (SC) Theme (eYRC 2026-27).
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
# Filename:		    task1a_boilerplate.py
# Functions:
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw, /etc... ]


################### IMPORT MODULES #######################

from matplotlib import image
import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

##################### TASK CONSTANTS #######################

# Two ores of each type are spawned - six in all - told apart by an id of 1 or 2.
ore_types = ['azurite_ore', 'malachite_ore', 'vanadinite_ore']

# The RealSense topics. The depth image is ALIGNED to the colour image.
color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'
COLOR_DETECTION_DICT = {
    'vanadinite_ore': {
        'lower': np.array([0, 180, 180]),
        'upper': np.array([17, 255, 255])
    },
    'azurite_ore': {
        'lower': np.array([100, 20, 30]),
        'upper': np.array([140, 255, 255])
    },
    'malachite_ore': {
        'lower': np.array([60, 150, 140]),
        'upper': np.array([75, 255, 255])
    }
}

# The parent frame every ore transform is published against.
base_frame = 'base_link'


##################### FUNCTION DEFINITIONS #######################

def detect_ores(image):
    '''
    Description:    Function to detect the ores present in a colour image frame and
                    return the pixel location and the type of each one found.

    Args:
        image                   (Image):    Input colour image frame received from the camera topic

    Returns:
        center_ore_list         (list):     Center pixel (cX, cY) of every ore detected in the frame
        ore_type_list           (list):     Type of each ore detected, taken from 'ore_types'
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    center_ore_list = []
    ore_type_list = []

    ############ ADD YOUR CODE HERE ############
    hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    ore_id_counter = {name: 0 for name in COLOR_DETECTION_DICT} 
    for color_name, bounds in COLOR_DETECTION_DICT.items():
        mask = cv2.inRange(hsv, bounds['lower'], bounds['upper'])

        open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        cleaned_mask = cv2.morphologyEx (mask,cv2.MORPH_OPEN,open_kernel)
        final_mask = cv2.morphologyEx(cleaned_mask  , cv2.MORPH_CLOSE, close_kernel)

        contour,hierarcy = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cont in contour:
            if cv2.contourArea(cont) < 120:
                continue
            M = cv2.moments(cont)
            cX = int(M['m10'] / M['m00'])
            cY = int(M['m01'] / M['m00'])
            ore_id_counter[color_name] += 1
            ore_label = f'{color_name}_{ore_id_counter[color_name]}'
            
            x, y, w, h = cv2.boundingRect(cont)
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 255), 2)
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
            cv2.putText(image, color_name, (x, y-5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.imwrite("detected_ores.jpg", image)
            center_ore_list.append((cX, cY))
            ore_type_list.append(color_name)

    # INSTRUCTIONS & HELP :

    #	->  Detect the ores by COLOUR, and return a center pixel and a type for each.
    #       ->  HINT: hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #                 mask = cv2.inRange(hsv, lower, upper)       # one pair per type
    #                 H is 0-179, S and V are 0-255. The frame is BGR, not RGB.

    #   ->  Read the bounds off a saved frame rather than copying them from a tutorial.

    #   ->  Clean the mask, and drop anything too small to be an ore.
    #       ->  HINT: cv2.morphologyEx (MORPH_OPEN, MORPH_CLOSE), cv2.contourArea

    #   ->  Reduce each region to one center pixel.
    #       ->  HINT: M  = cv2.moments(contour)                   # guard against m00 == 0
    #                 cX = int(M['m10'] / M['m00'])
    #                 cY = int(M['m01'] / M['m00'])

    #   ->  Draw your detections on the frame while you are developing.
    #       ->  HINT: cv2.circle, cv2.putText

    ############################################

    return center_ore_list, ore_type_list


##################### CLASS DEFINITION #######################

class ore_tf(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to detect the ores in the cell and
                    broadcast a transform for each one.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_tf
        '''

        super().__init__('ore_tf_publisher')                                            # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, color_topic, self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, depth_topic, self.depthimagecb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.caminfocb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)

        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())
        self.cam_info = None                                                            # camera intrinsics variable (from caminfocb())


        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :
        #	->  Add any variable your detection needs to keep between frames.
        #       ->  HINT: The two ores of a type must keep their ids for the whole run, and
        #                 'detect_ores' returns them unordered.

        ############################################


    def depthimagecb(self, data):
        '''
        Description:    Callback function for the aligned depth camera topic.
                        Use this function to receive the depth image and convert it to a CV2 image.

        Args:
            data (Image):    Input depth image frame received from the aligned depth camera topic

        Returns:
        '''

        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        # print(self.depth_image.dtype)
        if self.depth_image is None:
            return
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Convert the ROS Image message to a CV2 image and store it.
        #       ->  HINT: self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        #   ->  Get the units right. Print 'depth.dtype'.
        #       ->  HINT: 32FC1 is METRES; 16UC1 is MILLIMETRES, so divide by 1000.

        #   ->  Drop the pixels that carry no reading.
        #       ->  HINT: depth[np.isfinite(depth) & (depth > 0.0)]

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for the colour camera raw topic.
                        Use this function to receive the raw image and convert it to a CV2 image.

        Args:
            data (Image):    Input coloured raw image frame received from the image_raw camera topic

        Returns:
        '''
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        if self.cv_image is None:
            return
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Convert the ROS Image message to a CV2 image and store it.
        #       ->  HINT: self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        ############################################


    def caminfocb(self, data):
        '''
        Description:    Callback function for the camera info topic.
                        Use this function to receive the camera's intrinsic parameters.

        Args:
            data (CameraInfo):    Camera calibration published by the camera

        Returns:
        '''
        self.cam_info = data
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Store the focal lengths and the principal point. Read them from the topic;
        #       never hard-code them.
        #       ->  HINT: 'k' is the pinhole matrix flattened row by row-
        #                     k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = data.k[0]
        self.fy = data.k[4]
        self.cx = data.k[2]
        self.cy = data.k[5]
        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect the ores and publish a transform for
                        each one on its estimated position.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        if self.cv_image is None:
            return

        center_ore_list, ore_type_list = detect_ores(self.cv_image)

        print("Centers:", center_ore_list)
        print("Types:", ore_type_list)

        for (u, v), ore_label in zip(center_ore_list, ore_type_list):
            if self.depth_image is None:
                continue
            patch = self.depth_image[v-4:v+5, u-4:u+5]
            z  = np.median(patch[np.isfinite(patch) & (patch > 0.0)])
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy
            

            point_in_camera = PointStamped()
            point_in_camera.header.frame_id = 'camera_color_optical_frame'
            point_in_camera.point.x = x
            point_in_camera.point.y = y
            point_in_camera.point.z = z

            tf = self.tf_buffer.lookup_transform(base_frame, 
                                'camera_color_optical_frame', 
                                rclpy.time.Time())
            p  = do_transform_point(point_in_camera, tf)
        
            frame_id = 'base_link'
            child_frame_id = ore_label
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = frame_id
            t.child_frame_id = child_frame_id
            t.transform.translation.x = float(p.point.x)
            t.transform.translation.y = float(p.point.y)
            t.transform.translation.z = float(p.point.z)
            t.transform.rotation.w = 1.0
            self.br.sendTransform(t)

    
        # INSTRUCTIONS & HELP :

        #	->  Return early until both images and the camera info have arrived.

        #   ->  Get the ore centers and their types from 'detect_ores' defined above

        #   ->  Read the depth at each center pixel. Depth is ALIGNED to colour.
        #       ->  HINT: Take the MEDIAN of a small window, not the one pixel-
        #                     patch = self.depth_image[cY-4:cY+5, cX-4:cX+5]
        #                     z     = np.median(patch[np.isfinite(patch) & (patch > 0.0)])

        #   ->  Deproject the center pixel (u, v) and its depth z into a 3D point-
        #           x = (u - cx) * z / fx
        #           y = (v - cy) * z / fy
        #           z = z
        #       ->  HINT: That point is in the camera's OPTICAL frame - 'data.header.frame_id'.

        #   ->  Transform it into 'base_frame'-
        #           tf = self.tf_buffer.lookup_transform(
        #                    base_frame, <optical frame>, rclpy.time.Time())
        #           p  = do_transform_point(point_in_camera, tf)
        #       ->  HINT: PointStamped and tf2_geometry_msgs are not imported above, and the
        #                 lookup raises until the tree has filled in.

        #   ->  Correct for what you measured: the camera sees the ore's top face, and the
        #       ore is named by its middle.

        #   ->  Give each ore an id and keep it for the whole run.
        #       ->  HINT: Match a detection to the nearest ore of that type already named,
        #                 on the pixel rather than the depth.

        #   ->  Publish one transform per ore, using Geometry Message - TransformStamped
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = '<ore_type>_<id>'      Ex: azurite_ore_1, where azurite_ore
        #                                                   is one of 'ore_types' and the id is 1 or 2
        #       ->  HINT: t.header.stamp, t.header.frame_id, t.child_frame_id,
        #                 t.transform.translation.x/.y/.z, t.transform.rotation.w = 1.0,
        #                 then self.br.sendTransform(t). Every number must be a float.
        #       ->  NOTE: Only the translation is read; the names are matched exactly.

        #   ->  Broadcast every ore on every cycle, not once.

        #   ->  Show the frame with your detections drawn on it using 'cv2.imshow'.

        ############################################


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the ore_tf
                    class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('ore_tf_process')                      # creating ROS node

    node.get_logger().info('Node created: Ore tf process')          # logging information

    ore_tf_class = ore_tf()                                         # creating a new object for class 'ore_tf'

    rclpy.spin(ore_tf_class)                                        # spining on the object to make it alive in ROS 2 DDS

    ore_tf_class.destroy_node()                                     # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()
