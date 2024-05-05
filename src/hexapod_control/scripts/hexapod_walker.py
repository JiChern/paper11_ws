#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import time
import math

from std_msgs.msg import Float32MultiArray
from hexapod import Hexapod

sys.path.append('/home/jichen/paper11_ws/src/hexapod/scripts')
from robot_interface import RobotInterface

from std_msgs.msg import Float32MultiArray


from gait_generator import GaitGenerator
from bezier import Bezier
from data_saver import DataSaver


PI = 3.14159265
TURN_TOL = 0.005
TURN_SLOPE = 0.8
SPEED_SLOPE = 0.4
MAX_SPEED = 1.5
TURN_SLEW = 0.8
# IKSOLVER 
MAXITER = 100
TOLERANCE = 0.001
SS_TIME = 2


class HexWalker(object):
    def __init__(self):
        
        self.b2d_walk_up = Bezier()
        self.b2d_walk_down = Bezier()

        self.b2d_walk_up.addPoint(-0.83775,0)
        self.b2d_walk_up.addPoint(-1.11701,0)
        self.b2d_walk_up.addPoint(-1.39626,0)
        self.b2d_walk_up.addPoint(0,3.2)
        self.b2d_walk_up.addPoint(1.39626,0)
        self.b2d_walk_up.addPoint(1.11701,0)
        self.b2d_walk_up.addPoint(0.83775,0)

        self.b2d_walk_down.addPoint(0.83775,0)
        self.b2d_walk_down.addPoint(-0.83775,0)


        self.interface = RobotInterface()

        self.joint_command = {'j_c1_lf':0, 'j_c1_lm':0.0, 'j_c1_lr':0, 
                         'j_c1_rf':0, 'j_c1_rm':0.0, 'j_c1_rr':0,
                         'j_thigh_lf':0, 'j_thigh_lm':0, 'j_thigh_lr':0,
                         'j_thigh_rr':0, 'j_thigh_rm':0, 'j_thigh_rf':0,
                         'j_tibia_lf':0, 'j_tibia_lm':0, 'j_tibia_lr':0,
                         'j_tibia_rr':0, 'j_tibia_rm':0, 'j_tibia_rf':0}


        self.turning = 0
        self.smoothturning = 0
        self.hexapod = Hexapod(self.interface)
        self.half_period = 1 
        self.legraise = 0.02

        self.standheight = -self.interface.leg1['rest_pos'][2]

        self.gait_sub = rospy.Subscriber('/gait', Float32MultiArray, self.gait_cb)
        self.phase = [0,0,0,0,0,0]

        self.mu = 0.7   # 0.7 metach; 0.2 cater

 
    
    def gait_cb(self,data):
        self.phase = data.data

    def leg_pose_from_phase(self, phase, dt):
        start_time = time.time()
        dt = 0.02
        turn_step = -(self.smoothturning - self.turning)
        if turn_step > TURN_SLEW*dt: turn_step = TURN_SLEW*dt
        if turn_step < -TURN_SLEW*dt: turn_step = -TURN_SLEW*dt
        self.smoothturning += turn_step
        

        # Turning Math
        if abs(self.smoothturning) <= TURN_TOL:
            turn_dist = math.tan((1.0-TURN_TOL)*PI/2.0)  #*50.0
        elif abs(self.smoothturning) > TURN_TOL:
            turn_dist = math.tan((1.0-self.smoothturning)*PI/2.0)   #*50.

        max_dist = 0.0
        for leg in self.interface.legs:
            dist = math.sqrt(pow(leg['rest_pos'][0]-turn_dist,2)+pow(leg['rest_pos'][1],2))
            if dist > max_dist: max_dist = dist
        
        step_size = 0.014

        max_sweep = -step_size/max_dist
        if turn_dist<0.0: max_sweep = -max_sweep

        # self.time += dt
        # if self.time%(2*self.fdf)<self.fdf: self.ground_group = 0
        # else: self.ground_group = 1 
        # print('GG:', self.ground_group)

        # print(phase)


        for index,leg in enumerate(self.interface.legs):
            start_time = time.time()


            cycle_time = phase[index]

            # if cycle_time < 0.5: 
            #     tht_pos, z_pos = self.b2d_walk_down.getPos(cycle_time/0.5)
            # else:
            #     tht_pos, z_pos = self.b2d_walk_up.getPos((cycle_time-0.5)/(0.5))
            if cycle_time < self.mu: 
                tht_pos, z_pos = self.b2d_walk_down.getPos(cycle_time/self.mu)
            else:
                tht_pos, z_pos = self.b2d_walk_up.getPos((cycle_time-self.mu)/(1-self.mu))

            tht_pos *= max_sweep

                # print('tht_pos: ', tht_pos)
                # print('z_pos: ', z_pos)

            # turn_dist = -turn_dist

            dist = math.sqrt(pow(turn_dist-leg['rest_pos'][0],2)+pow(leg['rest_pos'][1],2))

            tht0 = math.atan2(turn_dist-leg['rest_pos'][0], leg['rest_pos'][1])
            

            x_tar = turn_dist-dist*math.sin(tht_pos+tht0)
            y_tar = dist * math.cos(tht_pos+tht0)
            z_tar = -self.standheight + self.legraise*z_pos

                
            converged, jnt_angle = self.IKSolve(leg, np.array([x_tar,y_tar,z_tar]), dt)

            self.joint_command[leg['joint_names'][0]] = jnt_angle[0] 
            self.joint_command[leg['joint_names'][1]] = jnt_angle[1]
            self.joint_command[leg['joint_names'][2]] = jnt_angle[2]

        return self.joint_command

        

    def IKSolve(self, leg, target, dt):
        start_time = time.time()
        converged = False
        diff = 100
        iter = 0
        joint_angles, trans, _,jacobian = self.hexapod.fk(leg['interface'],leg['joint_names'])


        error = (target-trans).reshape(3,1)
        jacobian = jacobian[0:3,:]
        cart_vel = error
        pseudo = np.linalg.pinv(jacobian)
        q_dot = np.matmul(pseudo, cart_vel)

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
                break
            iter += 1


        # q_dot = np.clip(q_dot,-0.04,0.04)


        joint_angles[0] = joint_angles[0] + q_dot[0]*dt
        joint_angles[1] = joint_angles[1] + q_dot[1]*dt
        joint_angles[2] = joint_angles[2] + q_dot[2]*dt


        return converged, joint_angles

 

if __name__ == '__main__':
    rospy.init_node('test_gait')

    walker = HexWalker()

    data_folder = '/home/jichen/paper11_ws/src/hexapod_control/scripts/motor_data'
    ds = DataSaver(data_folder)
    


    hz = 200
    
    r = rospy.Rate(hz)

    dt = 1/hz

    start_time = time.time()


    while not rospy.is_shutdown():
        loop_start_time = time.time()
        duration = time.time()-start_time

        jc = walker.leg_pose_from_phase(walker.phase,dt)
        walker.hexapod.exec_joint_command(jc)

        data = [duration, jc['j_c1_lf'],jc['j_c1_lm'],jc['j_c1_lr'],
                          jc['j_c1_rf'],jc['j_c1_rm'],jc['j_c1_rr'],
                          jc['j_thigh_lf'],jc['j_thigh_lm'],jc['j_thigh_lr'],
                          jc['j_thigh_rf'],jc['j_thigh_rm'],jc['j_thigh_rr'],
                          jc['j_tibia_lf'],jc['j_tibia_lm'],jc['j_tibia_lr'],
                          jc['j_tibia_rf'],jc['j_tibia_rm'],jc['j_tibia_rr']]
        

        # data = [duration, jc['j_c1_lf']]

        ds.dump_data(data)



        # r.sleep()

        print('duration', duration)

    # plt.figure()

    # plt.subplot(6,1,1)
    # plt.plot(duration_vec,z_vec0)
    # plt.subplot(6,1,2)
    # plt.plot(duration_vec,z_vec1)
    # plt.subplot(6,1,3)
    # plt.plot(duration_vec,z_vec2)
    # plt.subplot(6,1,4)
    # plt.plot(duration_vec,z_vec3)
    # plt.subplot(6,1,5)
    # plt.plot(duration_vec,z_vec4)
    # plt.subplot(6,1,6)
    # plt.plot(duration_vec,z_vec5)

    # plt.show()


    