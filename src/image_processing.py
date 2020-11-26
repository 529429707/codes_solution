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


class image_processor:
    def __init__(self):
       # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)

        # set up history data of joints positions for each joint 
        # in case the joints can not be viewd fully by the cameras
        self.yellow = np.zeros(4)
        self.blue = np.zeros(4)
        self.green = np.zeros(4)
        self.red = np.zeros(4)

        # initialize subscribers to get joints positions from camera1 and camera2 and use callback functions
        self.joint_cam1 = rospy.Subscriber('joints_pos_camera1', Float64MultiArray, self.callback1)
        self.joint_cam2 = rospy.Subscriber('joints_pos_camera2', Float64MultiArray, self.callback2)
        # initiaslize publishers to send estimated joints angles
        self.joints_2_pub = rospy.Publisher("joints_2_angle_pos", Float64, queue_size=10)
        self.joints_3_pub = rospy.Publisher("joints_3_angle_pos", Float64, queue_size=10)
        self.joints_4_pub = rospy.Publisher("joints_4_angle_pos", Float64, queue_size=10)


    def callback1(self,pos):
        # reshape the positions with each row corresponds to a joint
        camera1_data = np.reshape(pos.data, [3, 2])

        # get coordinate on y-axis for each joint
        self.blue[1] = -camera1_data[0][0]
        self.green[1] = -camera1_data[1][0]
        self.red[1] = -camera1_data[2][0]

        # get coordinate on z-axis on camera1 for each joint
        self.blue[2] = camera1_data[0][1]
        self.green[2] = camera1_data[1][1]
        self.red[2] = camera1_data[2][1]

    def callback2(self,pos):
        # reshape the positions with each row corresponds to a joint
        camera2_data = np.reshape(pos.data, [3, 2])

        # get coordinate on x-axis for each joint and targets
        self.blue[0] = -camera2_data[0][0]
        self.green[0] = -camera2_data[1][0]
        self.red[0] = -camera2_data[2][0]

        # get coordinate on z-axis on camera2 for each joint and targets
        self.blue[3] = camera2_data[0][1]
        self.green[3] = camera2_data[1][1]
        self.red[3] = camera2_data[2][1]

        self.angle_estimation()

    # use joint coordinates to find the joint angles and publish the angles
    def angle_estimation(self):
        # using joint positions to find joint angles
        joint_2_angle, joint_3_angle, joint_4_angle = self.estimate_joint_angles()

        # assign each angle to data
        self.joint2 = Float64()
        self.joint2.data = joint_2_angle
        self.joint3 = Float64()
        self.joint3.data = joint_3_angle
        self.joint4 = Float64()
        self.joint4.data = joint_4_angle
        
        # print the estimated joints coordinates
        print('blue joint position:' , self.blue)
        print('green joint position:' , self.green)
        print('red end-effector position:' , self.red)
        print('-'*20)

        try:
            # publish the estimated angles 
            self.joints_2_pub.publish(self.joint2)
            self.joints_3_pub.publish(self.joint3)
            self.joints_4_pub.publish(self.joint4)
            
        except CvBridgeError as e:
            print(e)


    def estimate_joint_angles(self):
        joint_2_angle = -np.arctan2(self.green[1] - self.blue[1], self.green[2] - self.blue[2])
        joint_3_angle =  np.arctan2(self.green[0] - self.blue[0], self.green[3] - self.blue[3])
        joint_4_angle = -np.arctan2(self.red[1] - self.green[1], self.red[2] - self.green[2])-joint_2_angle

        return joint_2_angle, joint_3_angle, joint_4_angle


# call the class
def main(args):
    ic = image_processor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)

