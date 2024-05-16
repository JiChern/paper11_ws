#!/usr/bin/env python3

import math as m
# from turtle import color

import numpy as np
import matplotlib.pyplot as  plt
from matplotlib.animation import FuncAnimation
import math
import rospy
from std_msgs.msg import Float32MultiArray, Int16
import time
import rospy
from bezier import Bezier2
from hexapod import Hexapod
import csv
import sys

sys.path.append('/home/jichen/paper11_ws/src/hexapod/scripts')
from robot_interface import RobotInterface

from std_msgs.msg import Float32MultiArray
from mpl_toolkits.mplot3d.axes3d import Axes3D

MAXITER = 10
TOLERANCE = 0.00005


class TrajGenerator(object):
    def __init__(self, omega_0, Hz):

        self.interface = RobotInterface()
        self.hexapod = Hexapod(self.interface)

        self.omega_0  = omega_0
        self.Hz = Hz
        self.mu = 0.7
        self.operator = 1
        self.disp_r = 0.02*self.operator
        self.v = self.omega_0*self.disp_r*2/(2*math.pi*self.mu)

        self.beziers = {'0':Bezier2(),'1':Bezier2(),'2':Bezier2(),
                        '3':Bezier2(),'4':Bezier2(),'5':Bezier2()}

        self.disp_l = [-0.02,-0.02,-0.02,-0.02,-0.02,-0.02]*self.operator

        self.y = [0,0,0,0,0,0]

        self.standheight = -self.interface.leg1['rest_pos'][2]

        self.joint_command = {'j_c1_lf':0, 'j_c1_lm':0.0, 'j_c1_lr':0, 
                    'j_c1_rf':0, 'j_c1_rm':0.0, 'j_c1_rr':0,
                    'j_thigh_lf':0, 'j_thigh_lm':0, 'j_thigh_lr':0,
                    'j_thigh_rr':0, 'j_thigh_rm':0, 'j_thigh_rf':0,
                    'j_tibia_lf':0, 'j_tibia_lm':0, 'j_tibia_lr':0,
                    'j_tibia_rr':0, 'j_tibia_rm':0, 'j_tibia_rf':0}

        self.stance_start_time = [0,0,0,0,0,0]
        self.last_state = [0,0,0,0,0,0]  # 0 means stance, 1 means swing

        self.leg_traj_x = {'1':[],'3':[],'5':[],'2':[],'4':[],'6':[]}
        self.leg_traj_y = {'1':[],'3':[],'5':[],'2':[],'4':[],'6':[]}
        self.leg_traj_z = {'1':[],'3':[],'5':[],'2':[],'4':[],'6':[]}

        self.duration_traj = {'1':[],'3':[],'5':[],'2':[],'4':[],'6':[]}

        self.t_traj = {'1':[],'3':[],'5':[],'2':[],'4':[],'6':[]}

        self.gait_sub = rospy.Subscriber('/gait', Float32MultiArray, self.gait_cb)
        self.phase = [0,0,0,0,0,0]

        self.time_last = [0,0,0,0,0,0]


    def calculate_stance_v(self):
        self.v = -self.omega_0*self.disp_r*2/(2*math.pi*self.mu)
        return self.v
    

    def leg_pose_from_phase(self,phase):

        for index,leg in enumerate(self.interface.legs):

            cycle_time = phase[index]

            if cycle_time < self.mu: # Stance Phase
                
                if self.last_state[index] == 1:
                    self.time_last[index] = time.time()
                    self.y[index] = self.disp_r

                v = self.calculate_stance_v()

                dt = time.time()-self.time_last[index]

                if index == 0:
                    self.t_traj['1'].append(dt)

                dt = 1/self.Hz
                dy = v*dt

                self.y[index] = self.y[index] + dy

                z_pos = 0
                self.last_state[index] = 0
                self.time_last[index] = time.time()

            else:
                if self.last_state[index] == 0:
                    x1 = self.y[index]
                    x2 = (self.y[index] + self.disp_r)/2
                    x3 = self.disp_r

                    y1 = 0
                    y2 = 0.02
                    y3 = 0

                    x_vec = [x1,x2,x3]
                    y_vec = [y1,y2,y3]
                    
                    self.beziers[str(index)].setPoint(x_vec, y_vec)
                    if index == 0:
                        print('bezier x: ', self.beziers[str(index)].x_pos, ' x_vec: ',x_vec)
                        print('bezier y: ', self.beziers[str(index)].y_pos, ' y_vec: ',y_vec)

                self.y[index], z_pos = self.beziers[str(index)].getPos((cycle_time-self.mu)/(1-self.mu))

                self.last_state[index] = 1

            x_tar = leg['rest_pos'][0]
            y_tar = leg['rest_pos'][1] + self.y[index] 
            z_tar = -self.standheight + z_pos     

            converged, jnt_angle = self.IKSolve(leg, np.array([x_tar,y_tar,z_tar]))

            if converged:
                # print('converged!!')
                self.joint_command[leg['joint_names'][0]] = jnt_angle[0] 
                self.joint_command[leg['joint_names'][1]] = jnt_angle[1]
                self.joint_command[leg['joint_names'][2]] = jnt_angle[2]


            # Record Data

            self.leg_traj_x[leg['id']].append(x_tar)
            self.leg_traj_y[leg['id']].append(y_tar)
            self.leg_traj_z[leg['id']].append(z_tar)

        return self.joint_command


    def IKSolve(self, leg, target):
        self.leg_traj_x[leg['id']].append(target[0])
        self.leg_traj_y[leg['id']].append(target[1])
        self.leg_traj_z[leg['id']].append(target[2])

        converged = False
        diff = 100
        iter = 0
        joint_angles, trans, _,jacobian = self.hexapod.fk(leg['interface'],leg['joint_names'])


        while iter < MAXITER:
            
            error = (target-trans).reshape(3,1)
            jacobian = jacobian[0:3,:]
            cart_vel = error 
            pseudo = np.linalg.pinv(jacobian)
            q_dot = np.matmul(pseudo, cart_vel)

            joint_angles[0] = joint_angles[0] + q_dot[0]*0.5
            joint_angles[1] = joint_angles[1] + q_dot[1]*0.5
            joint_angles[2] = joint_angles[2] + q_dot[2]*0.5

            trans,_,_ = leg['interface'].forward_kinematics(joint_angles)

            jacobian = leg['interface'].jacobian(joint_angles)
            diff = math.sqrt(math.pow(target[0]-trans[0],2)+math.pow(target[1]-trans[1],2)+
                            math.pow(target[2]-trans[2],2))
            
            

            if (diff<TOLERANCE):
                converged = True
                # print("converged!!")
                break

            iter += 1

            


        # q_dot = np.clip(q_dot,-0.04,0.04)


        return converged, joint_angles


    def gait_cb(self,data):
        self.phase = data.data


if __name__ == '__main__':
    rospy.init_node('traj_generator')

    hz = 100

    tg = TrajGenerator(omega_0=np.pi*2, Hz=hz)
    
    r = rospy.Rate(hz)


    start_time = time.time()

    tg.time_last = np.ones(6)*time.time()

    slp_time = 1/hz  

    while not rospy.is_shutdown():
        duration = time.time()-start_time

        jc = tg.leg_pose_from_phase(tg.phase)
        
        a = 1

        tg.hexapod.exec_joint_command(jc)

        time.sleep(slp_time-(time.time()%slp_time))

    fig = plt.figure()
 

    # plt.plot(tg.leg_traj_y['1'],tg.leg_traj_z['1'])


    ax = Axes3D(fig)



    ax.plot3D(tg.leg_traj_x['1'], tg.leg_traj_y['1'],tg.leg_traj_z['1'])
    ax.plot3D(tg.leg_traj_x['2'], tg.leg_traj_y['2'],tg.leg_traj_z['2'])
    ax.plot3D(tg.leg_traj_x['3'], tg.leg_traj_y['3'],tg.leg_traj_z['3'])
    ax.plot3D(tg.leg_traj_x['4'], tg.leg_traj_y['4'],tg.leg_traj_z['4'])
    ax.plot3D(tg.leg_traj_x['5'], tg.leg_traj_y['5'],tg.leg_traj_z['5'])
    ax.plot3D(tg.leg_traj_x['6'], tg.leg_traj_y['6'],tg.leg_traj_z['6'])

    plt.show()
    