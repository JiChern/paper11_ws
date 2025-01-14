#!/usr/bin/env python3

import rospy
import numpy as np
import time
import math
import matplotlib.pyplot as plt

from brain import HexBrain
from hexapod_walker import HexWalker
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
             'cater':{'theta':CATER, 'mu':0.2},   #
             'metach':{'theta':METACH, 'mu':0.7},
             'wave':{'theta':WAVE, 'mu':0.83},
             'tetra':{'theta':TETRA, 'mu':0.66},
             'hsmetach':{'theta':METACH, 'mu':0.4}}


if __name__ == '__main__':
    print(TETRA)

    rospy.init_node('gtexp2')
    walker = HexWalker()
    brain = HexBrain(walker)
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
    mu_vec = []


    brain.walker.mu = gait_dict['metach']['mu']
    print(gait_dict['cater']['mu'])
    print(brain.walker.mu)

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
            smooth_theta = brain.gait_transition('metach', 'cater', progress)

        if 30<duration<40.5:
            brain.ts = 10
            trans_start_time = 30
            progress = duration-trans_start_time
            smooth_theta = brain.gait_transition('cater', 'tri', progress)

        print('duration: ',duration, ' mu: ', brain.walker.mu)

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
