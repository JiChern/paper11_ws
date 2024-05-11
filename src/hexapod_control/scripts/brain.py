#!/usr/bin/env python3

import rospy
import math
import numpy as np
from gait_generator import GaitGenerator
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
from bezier import Bezier
from std_msgs.msg import Float32MultiArray
import sys

from hexapod_walker import HexWalker
import csv
from bezier import Bezier
from data_saver import DataSaver

pi = math.pi
tpi = math.pi*2

""" 6 cells """
CATER = tpi/3*np.ones(6)
TRI = pi*np.ones(6)
METACH = tpi/6*np.ones(6)
WAVE = np.array([tpi/3,tpi/3, pi, tpi/3,tpi/3, pi/3])
TETRA = np.array([tpi/3,tpi/3,0,tpi/3,tpi/3,2*tpi/3])


gait_dict = {'tri':{'theta':TRI, 'mu':0.5},
             'cater':{'theta':CATER, 'mu':0.2},
             'metach':{'theta':METACH, 'mu':0.7},
             'wave':{'theta':WAVE, 'mu':0.83},
             'tetra':{'theta':TETRA, 'mu':0.66}}

class HexBrain(object):
    def __init__(self):
        self.beta = 0.995
        self.theta_pub = rospy.Publisher('/theta_command', Float32MultiArray, queue_size=10)
        self.theta_sub = rospy.Subscriber('/current_theta', Float32MultiArray, self.theta_cb, queue_size=10)
        self.theta = np.zeros(6)
        self.kai = 1
        self.ts = 5

        self.bezier = Bezier()
        self.bezier.addPoint(0,0)
        self.bezier.addPoint(0.5,0)
        self.bezier.addPoint(0.5,1)
        self.bezier.addPoint(1,1)

        self.walker = HexWalker()

        self.in_trans = False
        self.trans_finish = False

    def theta_cb(self, data):
        self.theta = np.array(data.data)

    def smooth_theta(self, start_theta, target_theta, progress):
        if progress < self.ts:
            t = progress/self.ts
            _,percent = self.bezier.getPos(t)
            print('theta_percent: ', percent)
            print('start_theta: ', start_theta)
            print('target_theta: ', target_theta)

            smooth_theta = start_theta + (target_theta - start_theta)*percent
        else:
            smooth_theta = target_theta

        return smooth_theta
    
    def smooth_mu(self, start_mu, target_mu, progress):
        if progress < self.ts:
            t = progress/self.ts
            _,percent = self.bezier.getPos(t)
            smooth_mu = start_mu + (target_mu - start_mu)*percent
        else:
            smooth_mu = target_mu

        return smooth_mu

    
    def gait_transition(self, current_gait, target_gait):
        
        self.trans_finish = False
        smooth_theta = np.zeros(6)

        if not self.in_trans:
            # Execute this function, start the transition, 
            self.t_start_time = time.time()
            self.in_trans = True
        else:                                            
            progress = time.time()-self.t_start_time
            current_theta = gait_dict[current_gait]['theta']
            current_mu = gait_dict[current_gait]['mu']
            target_theta = gait_dict[target_gait]['theta']
            target_mu = gait_dict[target_gait]['mu']

            smooth_theta = self.smooth_theta(current_theta, target_theta,progress)
            smooth_mu = self.smooth_mu(current_mu, target_mu,progress)

            msg = Float32MultiArray()
            msg.data = smooth_theta
            self.theta_pub.publish(msg)
            self.walker.mu = smooth_mu

            if progress > self.ts:
                msg = Float32MultiArray()
                msg.data = target_theta
                self.theta_pub.publish(msg)
                self.walker.mu = target_mu
                self.in_trans = False

        return smooth_theta
    
    def stable_gait(self, theta):
        theta_dict = {'tri':TRI, 'cater':CATER, 'metach':METACH, 'wave':WAVE, 'tetra':TETRA}

        res_key, res_val = min(theta_dict.items(), key=lambda x: np.linalg.norm(theta - x[1]))
        return res_key


if __name__ == '__main__':
    print(TETRA)


    rospy.init_node('test_gait_transition')

    brain = HexBrain()
    data_folder = '/home/jichen/paper11_ws/src/hexapod_control/scripts/motor_data'
    ds = DataSaver(data_folder,2)

    hz = 200
    
    r = rospy.Rate(hz)

    dt = 1/hz

    start_time = time.time()

    mu = 0.5
    mu_vec = []

    percentage = 0

    # msg = rospy.wait_for_message('current_theta',Float32MultiArray)
    # current_theta = np.array(msg.data)

    # stable_gait = brain.stable_gait(current_theta)
    # theta1 = gait_dict[stable_gait]['theta']
    # mu1 = gait_dict[stable_gait]['mu']
    # walker.mu = mu1

    progress = 0

    theta_vec = []
    duration_vec = []

    brain.walker.mu = gait_dict['wave']['mu']

    while not rospy.is_shutdown():
        loop_start_time = time.time()
        duration = time.time()-start_time


        jc = brain.walker.leg_pose_from_phase(brain.walker.phase,dt)
        brain.walker.hexapod.exec_joint_command(jc)

        data = [duration, jc['j_c1_lf'],jc['j_c1_lm'],jc['j_c1_lr'],
                          jc['j_c1_rf'],jc['j_c1_rm'],jc['j_c1_rr'],
                          jc['j_thigh_lf'],jc['j_thigh_lm'],jc['j_thigh_lr'],
                          jc['j_thigh_rf'],jc['j_thigh_rm'],jc['j_thigh_rr'],
                          jc['j_tibia_lf'],jc['j_tibia_lm'],jc['j_tibia_lr'],
                          jc['j_tibia_rf'],jc['j_tibia_rm'],jc['j_tibia_rr']]

        ds.dump_data(data)

        if 10<duration<15 :
            smooth_theta = brain.gait_transition('wave', 'tetra')

        if 25<duration<30:
            smooth_theta = brain.gait_transition('tetra', 'tri')

        print('duration: ',duration)

        # r.sleep()
        dt = time.time()-loop_start_time

    # plt.figure()
    # plt.plot(duration_vec, theta_vec)
    # plt.show()


