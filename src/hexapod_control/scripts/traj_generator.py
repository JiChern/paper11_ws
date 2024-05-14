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

import csv
import sys

sys.path.append('/home/jichen/paper11_ws/src/hexapod/scripts')
from robot_interface import RobotInterface

from std_msgs.msg import Float32MultiArray



class TrajGenerator(object):
    def __init__(self, omega_0):

        self.interface = RobotInterface()

        self.omega_0  = omega_0
        self.mu = 0.5
        self.disp_r = 0.02
        self.v = self.omega_0*self.disp_r*2/(2*math.pi*self.mu)

        self.beziers = {'0':Bezier2(),'1':Bezier2(),'2':Bezier2(),
                        '3':Bezier2(),'4':Bezier2(),'5':Bezier2()}

        self.disp_l = [-0.02,-0.02,-0.02,-0.02,-0.02,-0.02]

        self.y = [0,0,0,0,0,0]

        # for index,leg in enumerate(self.interface.legs):
        #     self.target_y[index] = leg['rest_pos'][1]

        self.stance_start_time = [0,0,0,0,0,0]
        self.last_state = [0,0,0,0,0,0]  # 0 means stance, 1 means swing


    def calculate_stance_v(self):
        self.v = -self.omega_0*self.disp_r*2/(2*math.pi*self.mu)
        return self.v
    

    def leg_pose_from_phase(self,phase,dt):

        for index,leg in enumerate(self.interface.legs):
            start_time = time.time()


            cycle_time = phase[index]

            if cycle_time < self.mu: # Stance Phase
                if self.last_state[index] == 1:
                    self.stance_start_time[index] = time.time()
                    self.y[index] = self.disp_r
                v = self.calculate_stance_v()

                dy = v*(time.time*self.stance_start_time[index])
                self.y[index] += dy
                self.last_state[index] = 0

            else:
                pass
                # self.beziers[str(index)].



                tht_pos, z_pos = self.b2d_walk_up.getPos((cycle_time-self.mu)/(1-self.mu))
                self.last_state[index] = 1

            

                # print('tht_pos: ', tht_pos)
                # print('z_pos: ', z_pos)

            # turn_dist = -turn_dist

        #     dist = math.sqrt(pow(turn_dist-leg['rest_pos'][0],2)+pow(leg['rest_pos'][1],2))

        #     tht0 = math.atan2(turn_dist-leg['rest_pos'][0], leg['rest_pos'][1])
            

        #     x_tar = turn_dist-dist*math.sin(tht_pos+tht0)
        #     y_tar = dist * math.cos(tht_pos+tht0)
        #     z_tar = -self.standheight + self.legraise*z_pos
                
        #     # if index == 0:
                
        #     converged, jnt_angle = self.IKSolve(leg, np.array([x_tar,y_tar,z_tar]))

        #     if converged:
        #         # print('converged!!')
        #         self.joint_command[leg['joint_names'][0]] = jnt_angle[0] 
        #         self.joint_command[leg['joint_names'][1]] = jnt_angle[1]
        #         self.joint_command[leg['joint_names'][2]] = jnt_angle[2]

        # return self.joint_command

    # def leg_pose_from_phase1(self, phase, dt):
    #     start_time = time.time()

    #     turn_step = -(self.smoothturning - self.turning)
    #     if turn_step > TURN_SLEW*dt: turn_step = TURN_SLEW*dt
    #     if turn_step < -TURN_SLEW*dt: turn_step = -TURN_SLEW*dt
    #     self.smoothturning += turn_step


    #     # Turning Math
    #     if abs(self.smoothturning) <= TURN_TOL:
    #         turn_dist = math.tan((1.0-TURN_TOL)*PI/2.0)  #*50.0
    #     elif abs(self.smoothturning) > TURN_TOL:
    #         turn_dist = math.tan((1.0-self.smoothturning)*PI/2.0)   #*50.

    #     step_size = 0.02

    #     max_dist = 0.0
    #     for leg in self.interface.legs:
    #         dist = math.sqrt(pow(leg['rest_pos'][0]-turn_dist,2)+pow(leg['rest_pos'][1],2))
    #         if dist > max_dist: max_dist = dist
                

    #     max_sweep = -step_size/max_dist
    #     if turn_dist<0.0: max_sweep = -max_sweep

    #     # self.time += dt
    #     # if self.time%(2*self.fdf)<self.fdf: self.ground_group = 0
    #     # else: self.ground_group = 1 
    #     # print('GG:', self.ground_group)

    #     # print(phase)


    #     for index,leg in enumerate(self.interface.legs):
    #         start_time = time.time()


    #         cycle_time = phase[index]

    #         # if cycle_time < 0.5: 
    #         #     tht_pos, z_pos = self.b2d_walk_down.getPos(cycle_time/0.5)
    #         # else:
    #         #     tht_pos, z_pos = self.b2d_walk_up.getPos((cycle_time-0.5)/(0.5))
    #         if cycle_time < self.mu: 
    #             tht_pos, z_pos = self.b2d_walk_down.getPos(cycle_time/self.mu)
    #         else:
    #             tht_pos, z_pos = self.b2d_walk_up.getPos((cycle_time-self.mu)/(1-self.mu))

    #         tht_pos *= max_sweep

    #             # print('tht_pos: ', tht_pos)
    #             # print('z_pos: ', z_pos)

    #         # turn_dist = -turn_dist

    #         dist = math.sqrt(pow(turn_dist-leg['rest_pos'][0],2)+pow(leg['rest_pos'][1],2))

    #         tht0 = math.atan2(turn_dist-leg['rest_pos'][0], leg['rest_pos'][1])
            

    #         x_tar = turn_dist-dist*math.sin(tht_pos+tht0)
    #         y_tar = dist * math.cos(tht_pos+tht0)
    #         z_tar = -self.standheight + self.legraise*z_pos
                
    #         # if index == 0:
                
    #         converged, jnt_angle = self.IKSolve(leg, np.array([x_tar,y_tar,z_tar]))

    #         if converged:
    #             # print('converged!!')
    #             self.joint_command[leg['joint_names'][0]] = jnt_angle[0] 
    #             self.joint_command[leg['joint_names'][1]] = jnt_angle[1]
    #             self.joint_command[leg['joint_names'][2]] = jnt_angle[2]

    #     return self.joint_command
                
if __name__ == '__main__':
    rospy.init_node('traj_generator')
    tg = TrajGenerator(omega_0=2*math.pi)


