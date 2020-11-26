#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        self.bridge = CvBridge()
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.end_effector_x = rospy.Publisher("/end_effector_x_position", Float64, queue_size=10)
        self.end_effector_y = rospy.Publisher("/end_effector_y_position", Float64, queue_size=10)
        self.end_effector_z = rospy.Publisher("/end_effector_z_position", Float64, queue_size=10)

        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)

        # current joint angles
        self.joint_sub = rospy.Subscriber("/robot/joint_states", JointState, self.callback)

        # keep the integrated result
        self.joint_history = np.array([0.0, 0.0, 0.0, 0.0])

        # This is parsing the red joint position estimated by opencv using for control
        # We use forward kinematics instead so it is made into comment
        # self.red_joint_x = Float64
        # self.red_joint_y = Float64
        # self.red_joint_z = Float64
        # self.effector_x_sub = rospy.Publisher("/end_effector_x_position_by_cv", Float64, self.callback_red_x)
        # self.effector_y_sub = rospy.Publisher("/end_effector_y_position_by_cv", Float64, self.callback_red_y)
        # self.effector_z_sub = rospy.Publisher("/end_effector_z_position_by_cv", Float64, self.callback_red_z)

        self.time_trajectory = rospy.get_time()
        self.time_previous_step = np.array([rospy.get_time()], dtype='float64')
        self.error = np.array([0.0, 0.0, 0.0], dtype='float64')
        self.error_d = np.array([0.0, 0.0, 0.0], dtype='float64')

        # target1 position(sphere) estimated by computer vision
        self.target_x_actual = Float64
        self.target_y_actual = Float64
        self.target_z_actual = Float64
        self.target_x = rospy.Subscriber("/target1_x_position_by_cv", Float64, self.callbackx)
        rospy.sleep(0.1)
        self.target_y = rospy.Subscriber("/target1_y_position_by_cv", Float64, self.callbacky)
        rospy.sleep(0.1)
        self.target_z = rospy.Subscriber("/target1_z_position_by_cv", Float64, self.callbackz)
        rospy.sleep(0.1)


    # Getting the red joint position from cv
    # make into comment as we are using forward kinematics
    # def callback_red_x(self,data):
    #     self.red_joint_x = data
    # def callback_red_y(self,data):
    #     self.red_joint_y = data
    # def callback_red_z(self,data):
    #     self.red_joint_z = data

    # get the target x position
    def callbackx(self, data):
        self.target_x_actual = data

    # get the target y position
    def callbacky(self, data):
        self.target_y_actual = data

    # get the target z position
    def callbackz(self, data):
        self.target_z_actual = data

    def callback1(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        im1 = cv2.imshow('window1', self.cv_image1)
        cv2.waitKey(1)

    def callback(self, data):
        # print(type(self.target_x_actual.data))
        if (type(self.target_x_actual.data) is not float):
            return
        if (type(self.target_y_actual.data) is not float):
            return
        if (type(self.target_z_actual.data) is not float):
            return

        # if (type(self.red_joint_x.data) is not float):
        #     return
        # if (type(self.red_joint_y.data) is not float):
        #     return
        # if (type(self.red_joint_z.data) is not float):
        #     return

        # Keep a record a integrated result method made into comment
        # self.current_joint[0] = self.joint_history[0]
        # self.current_joint[1] = self.joint_history[1]
        # self.current_joint[2] = self.joint_history[2]
        # self.current_joint[3] = self.joint_history[3]

        # Get the current joint angles from joint_states topic
        joint1 = data.position[0]
        joint2 = data.position[1]
        joint3 = data.position[2]
        joint4 = data.position[3]
        # control the robot using identical method as lab3
        pos,q = self.control_closed(joint1,joint2,joint3,joint4)

        # publish the joint angles estimated to move the robot
        self.robot_joint1_pub.publish(q[0])
        self.robot_joint2_pub.publish(q[1])
        self.robot_joint3_pub.publish(q[2])
        self.robot_joint4_pub.publish(q[3])

        # publish the end effector position
        self.end_effector_x.publish(pos[0])
        self.end_effector_y.publish(pos[1])
        self.end_effector_z.publish(pos[2])

        # Keep a record a integrated result method made into comment
        # self.joint_history = q

    # identical method as lab3
    def control_closed(self, joint1,joint2,joint3,joint4):
        # P gain
        K_p = np.array([[11, 0, 0], [0, 11, 0], [0, 0, 11]])
        # D gain
        K_d = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
        # estimate time step
        cur_time = np.array([rospy.get_time()])
        dt = cur_time - self.time_previous_step
        self.time_previous_step = cur_time

        pos = self.detect_red(joint1,joint2,joint3,joint4)

        # pos_d = self.target1_position
        pos_d = np.asarray([self.target_x_actual.data, self.target_y_actual.data, self.target_z_actual.data])
        # estimate derivative of error
        self.error_d = ((pos_d - pos) - self.error) / (dt + 1e-11)
        # estimate error
        self.error = pos_d - pos
        print(self.error)
        J_inv = np.linalg.pinv(self.Jacobian_Matrix(joint1, joint2, joint3, joint4))
        dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p,
                                                                      self.error.transpose())))
        original_angle = np.asarray([joint1, joint2, joint3, joint4])
        q_d = original_angle + (dt * dq_d)

        return pos, q_d

    # estimate the end effector position
    def detect_red(self,joint1,joint2,joint3,joint4):

        # This is returning the red joint position estimated by opencv using for control
        # We use forward kinematics instead so it is made into comment
        # return np.asarray([self.red_joint_x.data, self.red_joint_y.data, self.red_joint_z.data])

        # result from CV is very inaccurate in some point, so we we forward kinematics to estimate it.
        return self.Forward_Kinematics(joint1,joint2,joint3,joint4)

    # Forward Kinematics Calculation
    def Forward_Kinematics(self, joint1, joint2, joint3, joint4):

        return np.array([
            3 * np.sin(joint1) * np.sin(joint2) * np.cos(joint3) * np.cos(joint4) + 3.5 * np.sin(joint1) * np.sin(
                joint2) * np.cos(joint3) +
            3 * np.cos(joint1) * np.cos(joint4) * np.sin(joint3) + 3.5 * np.cos(joint1) * np.sin(joint3) + 3 * np.sin(
                joint1) * np.cos(joint2) * np.sin(joint4),

            -3 * np.cos(joint1) * np.sin(joint2) * np.cos(joint3) * np.cos(joint4) - 3.5 * np.cos(joint1) * np.sin(
                joint2) * np.cos(joint3) +
            3 * np.sin(joint1) * np.cos(joint4) * np.sin(joint3) + 3.5 * np.sin(joint1) * np.sin(joint3) - 3 * np.cos(
                joint1) * np.cos(joint2) * np.sin(joint4),

            3 * np.cos(joint2) * np.cos(joint3) * np.cos(joint4) + 3.5 * np.cos(joint2) * np.cos(joint3) - 3 * np.sin(
                joint2) * np.sin(joint4) + 2.5
        ])

    # Jacobian Matrix Calculation
    def Jacobian_Matrix(self, joint1, joint2, joint3, joint4):
        return np.array([

            [3 * np.cos(joint1) * np.sin(joint2) * np.cos(joint3) * np.cos(joint4) + 3.5 * np.cos(joint1) * np.sin(joint2) * np.cos(joint3)
             - 3 * np.sin(joint1) * np.cos(joint4) * np.sin(joint3) - 3.5 * np.sin(joint1) * np.sin(joint3) + 3 * np.cos(joint1) * np.cos(
                joint2) * np.sin(joint4),
             3 * np.sin(joint1) * np.cos(joint2) * np.cos(joint3) * np.cos(joint4) + 3.5 * np.sin(joint1) * np.cos(joint2) * np.cos(joint3)
             - 3 * np.sin(joint1) * np.sin(joint2) * np.sin(joint4),
             -3 * np.sin(joint1) * np.sin(joint2) * np.sin(joint3) * np.cos(joint4) - 3.5 * np.sin(joint1) * np.sin(joint2) * np.sin(joint3)
             + 3 * np.cos(joint1) * np.cos(joint4) * np.cos(joint3) + 3.5 * np.cos(joint1) * np.cos(joint3),
             -3 * np.sin(joint1) * np.sin(joint2) * np.cos(joint3) * np.sin(joint4) - 3 * np.cos(joint1) * np.sin(joint4) * np.sin(joint3)
             + 3 * np.sin(joint1) * np.cos(joint2) * np.cos(joint4)],

            [3 * np.sin(joint1) * np.sin(joint2) * np.cos(joint3) * np.cos(joint4) + 3.5 * np.sin(joint1) * np.sin(joint2) * np.cos(joint3) +
             3 * np.cos(joint1) * np.cos(joint4) * np.sin(joint3) + 3.5 * np.cos(joint1) * np.sin(joint3) + 3 * np.sin(joint1) * np.cos(
                joint2) * np.sin(joint4),
             -3 * np.cos(joint1) * np.cos(joint2) * np.cos(joint3) * np.cos(joint4) - 3.5 * np.cos(joint1) * np.cos(joint2) * np.cos(
                 joint3) +
             3 * np.cos(joint1) * np.sin(joint2) * np.sin(joint4),
             3 * np.cos(joint1) * np.sin(joint2) * np.sin(joint3) * np.cos(joint4) + 3.5 * np.cos(joint1) * np.sin(joint2) * np.sin(joint3)
             + 3 * np.sin(joint1) * np.cos(joint4) * np.cos(joint3) + 3.5 * np.sin(joint1) * np.cos(joint3),
             3 * np.cos(joint1) * np.sin(joint2) * np.cos(joint3) * np.sin(joint4) - 3 * np.sin(joint1) * np.sin(joint4) * np.sin(joint3) -
             3 * np.cos(joint1) * np.cos(joint2) * np.cos(joint4)],

            [0,
             -3 * np.cos(joint3) * np.cos(joint4) * np.sin(joint2) - 3.5 * np.cos(joint3) * np.sin(joint2) - 3 * np.sin(joint4) * np.cos(
                 joint2),
             -3 * np.sin(joint3) * np.cos(joint4) * np.cos(joint2) - 3.5 * np.sin(joint3) * np.cos(joint2),
             -3 * np.cos(joint3) * np.sin(joint4) * np.cos(joint2) - 3 * np.cos(joint4) * np.sin(joint2)]
        ])


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

