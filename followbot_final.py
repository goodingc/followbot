#!/usr/bin/env python
import cv2
import math
from math import radians, atan2
import random

import cv_bridge
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import LaserScan


class Follower:
    def __init__(self):
        """
        @author Callum
        """
        self.front_range = 0.5
        self.r = 0
        self.l = 0
        self.f = 0

        self.image = None
        self.camera_matrix = None
        self.image_width = None

        self.x = None
        self.y = None
        self.theta = None

        self.wandering = False
        self.wander_target = None

        self.switch_target = False 

        rospy.init_node('followbot')
        self.rate_limiter = rospy.Rate(10)
        self.bridge = cv_bridge.CvBridge()
        rospy.Subscriber('camera/rgb/image_raw', Image, self.handle_image)
        rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.handle_camera_info)
        rospy.Subscriber('scan', LaserScan, self.handle_sensor)
        rospy.Subscriber('odom', Odometry, self.handle_odom)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def run(self):
        """
        @author Callum
        """
        while not rospy.is_shutdown():
            self.tick_handler_blue()
            self.rate_limiter.sleep()

    def mask_image(self, image):
        """
        @author Oliver
        @param image: A WxHx3 BGR image representing the view of the camera
        @return: A boolean image where input pixels that are green are marked True in the output image and all others
        are marked False.
        """

        HSV_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Convert RGB image to HSV colour space
        Low_HSV = np.array([50, 0, 0], dtype='uint8')  # Bottom end of green hue range
        High_HSV = np.array([70, 255, 255], dtype='uint8')  # Top end of green hue range

        # Apply thresholding to convert to boolean image (white pixels =
        # green object, black = background)
        Thresholded_Image = cv2.inRange(HSV_image, Low_HSV, High_HSV)

        return Thresholded_Image
    
    def mask_image_blue(self, image):
        """
        @author Shanshan CAO
        @param image: A WxHx3 BGR image representing the view of the camera
        @return: A boolean image where input pixels that are blue are marked True in the output image and all others
        are marked False.
        """

        HSV_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Convert RGB image to HSV colour space
        Low_HSV = np.array([118, 0, 0], dtype='uint8')  # Bottom end of blue hue range
        High_HSV = np.array([125, 255, 255], dtype='uint8')  # Top end of blue hue range
        
        Thresholded_Image = cv2.inRange(HSV_image, Low_HSV, High_HSV)

        return Thresholded_Image


    def find_beacon(self, mask):
        """
        @author Oliver
        @param mask: A boolean image where the color or the beacon is True
        @return: The screen-space coordinates of the center of the beacon to home towards
        """
        Image, Contours, Hierarchy = cv2.findContours(mask, cv2.RETR_TREE,
                                                      cv2.CHAIN_APPROX_SIMPLE)  # Extract contours (edges) from binary image

        if len(Contours) == 0:  # Exit if no contours (edges) found in binary image
            return

        Moments = cv2.moments(
            Contours[np.argmax(map(cv2.contourArea, Contours))])  # Calculate moments from largest found contours

        X_Pos = Moments["m10"] / Moments["m00"]  # Calculate X position of centroid from moments
        Y_Pos = Moments["m01"] / Moments["m00"]  # Calculate Y position of centroid from moments

        return [X_Pos, Y_Pos]

    def angle_offset(self, beacon_coords):
        """
        @author Callum
        @param beacon_coords: The screen-space coordinates of the beacon
        @return: The angle offset between the front of the robot and the beacon
        """
        center_projection = np.matmul(np.linalg.inv(self.camera_matrix),
                                      np.array([beacon_coords[0], beacon_coords[1], 1]))

        half_fov = math.atan2(self.image_width, 2 * self.camera_matrix[0, 0])

        return math.atan(center_projection[0] * math.tan(half_fov))

    def move_robot(self, angle_offset):
        """
        @author Callum
        Moves the robot forwards in the direction of the beacon using the offset angle.
        @param angle_offset: The angle offset between the front of the robot and the beacon
        """
        self.pub.publish(Twist(Vector3(0.1, 0, 0), Vector3(0, 0, -angle_offset * 0.3)))

    def wander(self):
        """
        @author Mohamed
        Wanders randomly moving 3 meters in one direction, rotating, then 3 meters in another and so on
        The value of self.wandering should be False for the first time this is called after having performed homing
        behaviour and then True for all times after that.
        """
        # declaring linear velocity
        forward_vel = Twist()
        forward_vel.linear.x = 0.4
        # declaring angular velocity
        angular_vel = Twist()
        angular_vel.linear.x = 0
        angular_vel.angular.z = radians(45)
        rand_direc = random.randint(0, 80)
        for n in range(0, 75):  # moves forward 3 meters in 7.5 seconds
            self.pub.publish(forward_vel)
            self.avoid()
            if self.wandering == False:
                return
            self.rate_limiter.sleep()
        # turns for a random time between(0, 80) which is ewuivelat to turning between(0 dgrees - 360 degrees)
        for n in range(rand_direc):
            self.pub.publish(angular_vel)
            if self.wandering == False:
                return
            self.rate_limiter.sleep()

    def tick_handler(self):
        """
        @author Callum
        Handles the tick and coordinates all other behaviour. By default called at a rate of 10Hz.
        Should also handle setting self.wandering to False after changing from wandering behaviour to homing behaviour.
        @return:
        """
        if self.image is None or self.camera_matrix is None or self.image_width is None:
            return
        mask = self.mask_image(self.image)
        beacon_coords = self.find_beacon(mask)
        if beacon_coords is None:
            self.wander()
            return
        self.wandering = False
        angle_offset = self.angle_offset(beacon_coords)
        self.move_robot(angle_offset)
    
    def tick_handler_blue(self):
        """
        @author Shanshan CAO

        Handles the tick and coordinates all other behaviour. By default called at a rate of 10Hz.
        Should also handle setting self.wandering to False after changing from wandering behaviour to homing behaviour.
        @return:
        """
        if self.image is None or self.camera_matrix is None or self.image_width is None:
            return


        if self.f < 0.2:
            if self.switch_target == False:
                print('Found green Object\n')
            else:
                print('Found blue Object\n')

            self.switch_target = True

        
        if self.switch_target == False :
            mask = self.mask_image(self.image)
        else:
            mask = self.mask_image_blue(self.image)            

        beacon_coords = self.find_beacon(mask)

        if beacon_coords is None:
            print('Wandering\n')
            self.wander()
            return

        self.wandering = False

        if self.switch_target == False :
            print('Going towards green object\n')
        else:
            print('Going towards blue object\n')
            
        angle_offset = self.angle_offset(beacon_coords)
        self.move_robot(angle_offset)

    def handle_image(self, msg):
        """
        @author Oliver
        Sets self.image with the value of the BGR encoded image from msg
        @param msg: The message received on the camera/rgb/image_raw topic
        """
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def handle_camera_info(self, msg):
        """
        @author Callum
        Sets self.image_width as the width value from the message.
        Sets self.camera_matrix as the 3x3 camera matrix from msg.
        @param msg: The message received on the camera/rgb/camera_info topic
        """
        self.image_width = msg.width
        self.camera_matrix = np.reshape(msg.K, (3, 3))

    def handle_odom(self, msg):
        """
        @author Oliver
        Sets self.[theta, x, y] as their respective values parsed from msg
        @param msg: The message received on the odom topic
        """
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = yaw

    def handle_sensor(self, msg):
        """
        @author Mohamed
        """
        self.f = msg.ranges[0]
        self.r = msg.ranges[250]
        self.l = msg.ranges[70]
        msg.range_max = self.front_range

    def avoid(self):
        """
        @author Mohamed
        """
        angular_vel_r = Twist()
        angular_vel_r.linear.x = 0
        angular_vel_r.angular.z = radians(90)
        angular_vel_l = Twist()
        angular_vel_l.linear.x = 0
        angular_vel_l.angular.z = radians(-90)
        b = self.r < self.l
        while self.f < self.front_range:
            ang_vel = (angular_vel_l if b is True else angular_vel_r)
            self.pub.publish(ang_vel)
            self.rate_limiter.sleep()


Follower().run()
