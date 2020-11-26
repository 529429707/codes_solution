#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        # initialize a publisher to send joint positions found by camera image
        self.joints_pos1_pub = rospy.Publisher("joints_pos_camera1", Float64MultiArray, queue_size=10)

        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.time_trajectory = rospy.get_time()

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()


    # detect the centre of the red circle
    def detect_red(self, image):
        # Isolate the red colour in the image as a binary image
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        # Obtain the moments of the binary image
        M = cv2.moments(mask)
        if(M['m00'] == 0):
            return self.detect_green(image)
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / (M['m00']+ 0.01))# add a small bias to avoid zero division
        cy = int(M['m01'] / (M['m00']+ 0.01))
        return np.array([cx, cy])

    # detect the centre of the green circle
    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if(M['m00'] == 0):
            return self.detect_blue(image)
        cx = int(M['m10'] / (M['m00']+0.01))
        cy = int(M['m01'] / (M['m00']+0.01))
        return np.array([cx, cy])


    # detect the centre of the blue circle
    def detect_blue(self, image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if(M['m00'] == 0):
            return np.array([0,0])
        cx = int(M['m10'] / (M['m00']+0.01))
        cy = int(M['m01'] / (M['m00']+0.01))
        return np.array([cx, cy])


    # detect the centre of the yellow circle
    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if(M['m00'] == 0):
            return np.array([0,0])
        cx = int(M['m10'] / (M['m00']+0.01))
        cy = int(M['m01'] / (M['m00']+0.01))
        return np.array([cx, cy])


    # calculate the conversion from pixel to meter by using distance between yellow and blue joints
    def pixel2meter(self, image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_yellow(image)
        circle2Pos = self.detect_blue(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 2.5 / np.sqrt(dist)

    # y and z coordinates can be found by using image data from camera 1
    def detect_joint_pos(self,image):
        # get conversion ratio from pixel to meters
        a = self.pixel2meter(image)
        # get the base coordinates by using immobile yellow joint
        base = a * self.detect_yellow(image)
        # convert pixel coordinates to meter coordinates
        circle1Pos = base - a * self.detect_blue(image)
        circle2Pos = base - a * self.detect_green(image)
        circle3Pos = base - a * self.detect_red(image)

        return np.concatenate((circle1Pos, circle2Pos, circle3Pos))

    # Define a circular trajectory
    def trajectory_angle(self):
        # get current time
        cur_time = rospy.get_time() - self.time_trajectory
        angle2 = (np.pi / 2) * np.sin((np.pi / 15) * cur_time)
        angle3 = (np.pi / 2) * np.sin((np.pi / 18) * cur_time)
        angle4 = (np.pi / 2) * np.sin((np.pi / 20) * cur_time)
        return np.asarray([angle2,angle3,angle4])

    # Recieve data from camera 1, process it, and publish
    def callback1(self, data):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', self.cv_image1)

        im1 = cv2.imshow('window1', self.cv_image1)
        cv2.waitKey(1)

        # get the joints positions (only y and z from camera 1)
        positions = self.detect_joint_pos(self.cv_image1)
        self.joint_pos = Float64MultiArray()
        self.joint_pos.data = positions

        # set up angles for each moving joint to rotate the robot
        angles = self.trajectory_angle()
        self.joint2 = Float64()
        self.joint2.data = angles[0]
        self.joint3 = Float64()
        self.joint3.data = angles[1]
        self.joint4 = Float64()
        self.joint4.data = angles[2]

        # Publish the results
        try:
            # publish the image data
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
            # publish joint positions viewed by camera 1
            self.joints_pos1_pub.publish(self.joint_pos)

            # move the robot joints
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)

        except CvBridgeError as e:
                print(e)


# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)



