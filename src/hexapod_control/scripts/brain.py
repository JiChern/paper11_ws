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
from gazebo_msgs.msg import ModelStates

import csv


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
             'tetra':{'theta':TETRA, 'mu':0.66},
             'hsmetach':{'theta':METACH, 'mu':0.4}}

class HexBrain(object):
    def __init__(self):
        self.beta = 0.995
        self.theta_pub = rospy.Publisher('/theta_command', Float32MultiArray, queue_size=10)
        self.theta_sub = rospy.Subscriber('/current_theta', Float32MultiArray, self.theta_cb, queue_size=10)
        self.ms_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb, queue_size=10)
        self.theta = np.zeros(6)
        self.kai = 1
        self.ts = 10

        self.bezier = Bezier()
        self.bezier.addPoint(0,0)
        self.bezier.addPoint(0.5,0)
        self.bezier.addPoint(0.5,1)
        self.bezier.addPoint(1,1)

        self.walker = HexWalker()

        self.in_trans = False
        self.trans_finish = False

        """
        CSV files
        """
        f0 = open("/home/jichen/paper11_ws/src/hexapod_control/scripts/data/pos.csv", 'w')
        self.pos_writer = csv.writer(f0)

        f =  open("/home/jichen/paper11_ws/src/hexapod_control/scripts/data/vel.csv", 'w')
        self.vel_writer = csv.writer(f)
        
        f1 = open("/home/jichen/paper11_ws/src/hexapod_control/scripts/data/mu.csv", 'w')
        self.mu_writer = csv.writer(f1)


    def theta_cb(self, data):
        self.theta = np.array(data.data)

    def model_cb(self, data):
        self.model_position = [data.pose[1].position.x, data.pose[1].position.y, data.pose[1].position.z]
        self.model_vel = [data.twist[1].linear.x, data.twist[1].linear.y, data.twist[1].linear.z]

    def smooth_theta(self, start_theta, target_theta, progress):
        if progress < self.ts:
            t = progress/self.ts
            _,percent = self.bezier.getPos(t)

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

    
    def gait_transition(self, current_gait, target_gait, progress):
        
        self.trans_finish = False
        smooth_theta = np.zeros(6)
       
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
    ds = DataSaver(data_folder,1)

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
    mu_vec = []


    brain.walker.mu = gait_dict['metach']['mu']

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

        if 10<duration<20.5:
            trans_start_time = 10
            progress = duration-trans_start_time
            smooth_theta = brain.gait_transition('wave', 'tetra', progress)

        if 30<duration<40.5:
            trans_start_time = 30
            progress = duration-trans_start_time
            smooth_theta = brain.gait_transition('tetra', 'tri', progress)

        print('duration: ',duration)

        # duration_vec.append(duration)
        # mu_vec.append(brain.walker.mu)

        brain.mu_writer.writerow([duration, brain.walker.mu])
        brain.pos_writer.writerow([duration]+brain.model_position)
        brain.vel_writer.writerow([duration]+brain.model_vel)

        # r.sleep()
        dt = time.time()-loop_start_time

    plt.figure()
    plt.plot(duration_vec, mu_vec)
    plt.show()


