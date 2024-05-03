#!/usr/bin/env python3

import sys
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
import numpy as np


sys.path.append('/home/jichen/paper11_ws/src/hexapod/scripts')
from robot_interface import RobotInterface

from robot_interface import RobotInterface

import math


from ias_pykdl import KDLInterface


class Hexapod:
    """Client ROS class for manipulating Hexapod in Gazebo"""

    def __init__(self, robot_interface):
        # self.ns = ns
        self.joints = None
        self.angles = None
        self.robot_interface = robot_interface


        self._sub_joints = rospy.Subscriber('/hexapod/joint_states', JointState, self._cb_joints, queue_size=1)
        rospy.loginfo('Waiting for joints to be populated...')
        while not rospy.is_shutdown():
            if self.joints is not None:
                break
            rospy.sleep(0.1)
            rospy.loginfo('Waiting for joints to be populated...')
        rospy.loginfo('Joints populated')

        # self.pub_g1 = rospy.Publisher("/group1/joint_commands", Float32MultiArray, queue_size=10)
        # self.pub_g2 = rospy.Publisher("/group2/joint_commands", Float32MultiArray, queue_size=10)
        # self.pub_g3 = rospy.Publisher("/group3/joint_commands", Float32MultiArray, queue_size=10)

        rospy.loginfo('Creating joint command publishers')
        self._pub_joints = {}
        for j in self.joints:
            print('joint_names: ', j)
            p = rospy.Publisher('/hexapod/'+ j + '_position_controller/command', Float64, queue_size=1)
            self._pub_joints[j] = p
        #self._pub_joints = rospy.Publisher(ns + 'hexapod_joint_controllers/command', Float64MultiArray, queue_size=10)
        rospy.sleep(1)

    def fk(self, interface, joint_names):
        angles = self.get_angles()


        joint_angles = np.zeros(3,dtype=float)
        joint_angles[0] = angles[joint_names[0]]
        joint_angles[1] = angles[joint_names[1]]
        joint_angles[2] = angles[joint_names[2]]
        trans, _, quaternions = interface.forward_kinematics(joint_angles)

        jacobian = interface.jacobian(joint_angles)
        # jacobian = jacobian[2:5,:]
        return joint_angles, trans, quaternions, jacobian

    def exec_joint_command(self, joint_command):
        for j in self.joints:
            msg = Float64()
            msg.data = joint_command[j]
            self._pub_joints[j].publish(msg)


# 0.25, -1.089

    def reset_joint(self):
        joint_command = {'j_c1_lf':1.047, 'j_c1_lm':0.0, 'j_c1_lr':-1.047, 
                         'j_c1_rf':-1.047, 'j_c1_rm':0.0, 'j_c1_rr':1.047,
                         'j_thigh_lf':0.25, 'j_thigh_lm':0.25, 'j_thigh_lr':0.25,
                         'j_thigh_rr':0.25, 'j_thigh_rm':0.25, 'j_thigh_rf':0.25,
                         'j_tibia_lf':-1.089, 'j_tibia_lm':-1.089, 'j_tibia_lr':-1.089,
                         'j_tibia_rr':-1.089, 'j_tibia_rm':-1.089, 'j_tibia_rf':-1.089}

        # joint_command['j_thigh_lf']=0.573+0.3*math.sin(time.time())
        
        self.exec_joint_command(joint_command)
        


    def _cb_joints(self, msg):
        # print(msg)
        if self.joints is None:
            self.joints = msg.name
        self.angles = msg.position

    def get_angles(self):
        if self.joints is None:
            return None
        if self.angles is None:
            return None
        return dict(zip(self.joints, self.angles))


if __name__ == '__main__':
    rospy.init_node('robot', anonymous=True)
    hex_interface = RobotInterface()
    hexapod = Hexapod(hex_interface)
    

    r = rospy.Rate(50)
    while not rospy.is_shutdown():
        joint_angles, trans, quaternions, jacobian = hexapod.fk(hex_interface.leg3['interface'],hex_interface.leg3['joint_names'])
        # print('joint angles: ', joint_angles)
        print('trans: ', trans)
        hexapod.reset_joint()

        r.sleep()


