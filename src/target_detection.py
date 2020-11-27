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
    def __init__(self):
        rospy.init_node('target_detection', anonymous=True)

        # initialize subscribers to get image data from camera1 and camera2
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.image1_callback)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.image2_callback)

        # initailize publishers to send x, y and z coordinates of sphere target
        self.target1_x_pub = rospy.Publisher("/target1_x_position_by_cv", Float64, queue_size=10)
        self.target1_y_pub = rospy.Publisher("/target1_y_position_by_cv", Float64, queue_size=10)
        self.target1_z_pub = rospy.Publisher("/target1_z_position_by_cv", Float64, queue_size=10)

        # initailize publishers to send x, y and z coordinates of box target
        self.target2x_position_pub = rospy.Publisher("/target2_x_position_by_cv", Float64, queue_size=10)
        self.target2y_position_pub = rospy.Publisher("/target2_y_position_by_cv", Float64, queue_size=10)
        self.target2z_position_pub = rospy.Publisher("/target2_z_position_by_cv", Float64, queue_size=10)

        # initialize publishers to send x, y and z coordiates of end effector for the control problem
        self.effector_x_pub = rospy.Publisher("/end_effector_x_position_by_cv", Float64, queue_size=10)
        self.effector_y_pub = rospy.Publisher("/end_effector_y_position_by_cv", Float64, queue_size=10)
        self.effector_z_pub = rospy.Publisher("/end_effector_z_position_by_cv", Float64, queue_size=10)

        # set up history data of target positions in case the targets can not be viewd by the cameras
        self.target1_history = [0.0, 0.0, 0.0, 0,0]
        self.target2_history = [0.0, 0.0, 0.0, 0,0]

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        self.target1_x_pos = Float64()
        self.target1_y_pos = Float64()
        self.target1_z_pos = Float64()
        self.target2_x_pos = Float64()
        self.target2_y_pos = Float64()
        self.target2_z_pos = Float64()


    # Detecting the center of red end-effector
    def detect_red(self, image):
        mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if(M['m00'] == 0):
            return self.detect_green(image)
        # Calculate pixel coordinates for the centre of the blob
        cx = int(M['m10'] / (M['m00']+ 0.01)) # add a small bias to avoid zero division
        cy = int(M['m01'] / (M['m00']+ 0.01))
        return np.array([cx, cy])


    # Detecting the centre of the green circle
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


    # Detecting the centre of the blue circle
    def detect_blue(self, image):
        mask = cv2.inRange(image, (100, 0, 0), (255, 80, 80))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if(M['m00'] == 0):
            return np.array([0,0])
        cx = int(M['m10'] / (M['m00'] + 0.01))
        cy = int(M['m01'] / (M['m00'] + 0.01))
        return np.array([cx, cy])

    # Detecting the centre of the yellow circle
    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 100, 100), (0, 200, 200))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        M = cv2.moments(mask)
        if(M['m00'] == 0):
            return np.array([0,0])
        cx = int(M['m10'] / (M['m00'] + 0.01))
        cy = int(M['m01'] / (M['m00'] + 0.01))
        return np.array([cx, cy])

    # Calculate the conversion from pixel to meter by using distance between yellow and blue joints
    def pixel2meter(self, image):
        # Obtain the centre of each coloured blob
        circle1Pos = self.detect_blue(image)
        circle2Pos = self.detect_yellow(image)
        # find the distance between two circles
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        return 2.5 / np.sqrt(dist)


    def detect_target(self, image,target1=False):
        # target1 is a flag shows whether this function is for detecting the sphere (first) target

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # orange target
        low_hsv = np.array([11, 43, 46])
        high_hsv = np.array([25, 255, 255])
        mask = cv2.inRange(hsv,lowerb=low_hsv,upperb=high_hsv)

        kernel = np.ones((8, 8), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if target1:
                # it is the sphere target when number of contour greater than 6
                if (len(approx) > 6):

                    M = cv2.moments(contour)
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    return np.asarray([cx,cy])
            else:
                # it is the box target when when number of contour smaller than 6
                if (len(approx) <= 6):
                    # print(cv2.moments(contour))
                    M = cv2.moments(contour)
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    return np.asarray([cx, cy])

        # return None when the target can not viewd the target fully
        return None

    def detect_target_pos(self,image,cam1=True):
        # cam1 is a flag shows whether camera 1 is using to get the target position
        # when camera 1 is using, the targets y and z coordinates can be found
        # when camera 2 is using, the targets x and z coordinates can be found

        # get the  2 coordinates found by camera image for both targets
        sphere_pos = self.detect_target(image,True)
        box_pos = self.detect_target(image,False)

        if cam1:
            index1 = 1 # get y coordinates using camera1
            index2 = 2 # get z coordinates using camera1
        else:
            index1 = 0 # get x coordinates using camera2
            index2 = 3 # get z coordinates using camera2

        if sphere_pos is None:
            # when the target can not be viewd clearly, use previous position of the target
            sphere_pos = np.array([self.target1_history[index1], self.target1_history[index2]])
        else:
            self.target1_history[index1] = sphere_pos[0]
            self.target1_history[index2] = sphere_pos[1]

        if box_pos is None:
            # when the target can not be viewd clearly, use previous position of the target
            box_pos = np.array([self.target2_history[index1], self.target2_history[index2]])
        else:
            self.target2_history[index1] = box_pos[0]
            self.target2_history[index2] = box_pos[1]

        # get conversion ratio from pixel to meters
        a = self.pixel2meter(image)
        # get the base coordinates by using immobile yellow joint
        center = a * self.detect_yellow(image)
        # convert pixel coordinates to meter coordinates
        sphere_pos = center - a * sphere_pos
        box_pos = center - a * box_pos
        # get the position of end effector for the control problem
        end_effector_pos = center - a * self.detect_red(image)

        return sphere_pos, box_pos, end_effector_pos


    def image1_callback(self, data):
        try:
            # get image data from camera 1
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # using image data to find y and z coordinates of targets and end effector
        sphere_pos, box_pos , end_effector_pos= self.detect_target_pos(self.cv_image1,cam1=True)

        # publish the y coordinates and z coordiantes of the sphere target and square target
        self.target1_y_pos.data = -sphere_pos[0]
        self.target1_z_pos.data = sphere_pos[1]
        self.target2_y_pos.data = -box_pos[0]
        self.target2_z_pos.data = box_pos[1]

        self.target1_y_pub.publish(self.target1_y_pos)
        self.target1_z_pub.publish(self.target1_z_pos)
        self.target2y_position_pub.publish(self.target2_y_pos)
        self.target2z_position_pub.publish(self.target2_z_pos)

        # publish the y coordinate and z coordiante of the end-effector
        self.effector_y = Float64()
        self.effector_y.data = -end_effector_pos[0]
        self.effector_z = Float64()
        self.effector_z.data = end_effector_pos[1]

        self.effector_y_pub.publish(self.effector_y)
        self.effector_z_pub.publish(self.effector_z)


    def image2_callback(self, data):
        try:
            # get image data from camera 2
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # using image data to find x and z coordinates of targets and end effector
        sphere_pos, box_pos, end_effector_pos = self.detect_target_pos(self.cv_image2,cam1=False)

        # publish the x coordinates of the sphere target and square target
        self.target1_x_pos.data = -sphere_pos[0]
        self.target2_x_pos.data = -box_pos[0]

        self.target1_x_pub.publish(self.target1_x_pos)
        self.target2x_position_pub.publish(self.target2_x_pos)

        # publish the x coordinate of the end-effector
        self.effector_x = Float64()
        self.effector_x.data = -end_effector_pos[0]

        self.effector_x_pub.publish(self.effector_x)

# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


