#!/usr/bin/env python
import cv2, math
from random import random

import cv_bridge
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry


class Follower:
    def __init__(self):
        self.image = None
        self.camera_matrix = None
        self.image_width = None

        self.x = None
        self.y = None
        self.theta = None

        self.wandering = False
        self.wander_target = None

        rospy.init_node('followbot')
        self.rate_limiter = rospy.Rate(10)
        self.bridge = cv_bridge.CvBridge()
        rospy.Subscriber('camera/rgb/image_raw', Image, self.handle_image)
        rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.handle_camera_info)
        rospy.Subscriber('odom', Odometry, self.handle_odom)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            self.tick_handler()
            self.rate_limiter.sleep()

    def mask_image(self, image):
        """
        @param image: A WxHx3 BGR image representing the view of the camera
        @return: A boolean image where input pixels that are green are marked True in the output image and all others
        are marked False.
        """

    def find_beacon(self, mask):
        """
        @param mask: A boolean image where the color or the beacon is True
        @return: The screen-space coordinates of the center of the beacon to home towards
        """

    def angle_offset(self, beacon_coords):
        """
        @param beacon_coords: The screen-space coordinates of the beacon
        @return: The angle offset between the front of the robot and the beacon
        """

    def move_robot(self, angle_offset):
        """
        Moves the robot forwards in the direction of the beacon using the offset angle.
        @param angle_offset: The angle offset between the front of the robot and the beacon
        """

    def wander(self):
        """
        Wanders randomly moving 3 meters in one direction, rotating, then 3 meters in another and so on
        The value of self.wandering should be False for the first time this is called after having performed homing
        behaviour and then True for all times after that.
        """

    def tick_handler(self):
        """
        Handles the tick and coordinates all other behaviour. By default called at a rate of 10Hz.
        Should also handle setting self.wandering to False after changing from wandering behaviour to homing behaviour.
        @return:
        """

    def handle_image(self, msg):
        """
        Sets self.image with the value of the BGR encoded image from msg
        @param msg: The message received on the camera/rgb/image_raw topic
        """

    def handle_camera_info(self, msg):
        """
        Sets self.image_width as the width value from the message.
        Sets self.camera_matrix as the 3x3 camera matrix from msg.
        @param msg: The message received on the camera/rgb/camera_info topic
        """

    def handle_odom(self, msg):
        """
        Sets self.[theta, x, y] as their respective values parsed from msg
        @param msg: The message received on the odom topic
        """


Follower().run()
