#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import time
import pybullet_utils.bullet_client as bullet_client
import pybullet
import math
from std_msgs.msg import Float32MultiArray


sys.path.append('/home/jichen/paper11_ws/src/hexapod/scripts')
from robot_interface import RobotInterface
from hexapod_bullet import Hexapod
from std_msgs.msg import Float32MultiArray

# from .trajectory_generator import TrajectoryGenerator
# from leg_control_framework.scripts.trajectory_generator import TrajectoryGenerator
# from leg_control_framework.scripts.gait_generator import *
# from leg_control_framework.scripts.utils import low_pass
# from robot_envs.hexapod.hexapod import Hexapod
import csv


_IDENTITY_ORIENTATION = (0, 0, 0, 1)
import matplotlib.pyplot as plt
import pybullet_data
# from leg_control_framework.scripts.utils import get_stance_avg_f


class HexEnv(object):
    def __init__(self, gravity) -> None:

        self._pybullet_client = bullet_client.BulletClient(connection_mode=pybullet.GUI)

        self.robot = Hexapod(self._pybullet_client)

        pybullet.setGravity(0.0,0.0,gravity)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI,0)
        pybullet.setPhysicsEngineParameter(numSolverIterations=1500)

        """ Add curved plane"""
        shift = [0, 0, 0]
        meshScale = [1, 1,1]
        # fileName = "/home/jichen/hex_ctrl/src/robot_envs/ground/curved002.obj"
        fileName = "/home/jichen/hex_ctrl/src/robot_envs/ground/plane_ele_003.obj"
        fileNameCol = fileName

        # the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)

        visualShapeId = pybullet.createVisualShape(shapeType=pybullet.GEOM_MESH,
                                            fileName=fileName,
                                            rgbaColor=[1, 1, 1, 1],
                                            specularColor=[0.4, .4, 0],
                                            visualFramePosition=shift,
                                            meshScale=meshScale)
        collisionShapeId = pybullet.createCollisionShape(shapeType=pybullet.GEOM_MESH,
                                                fileName=fileNameCol,
                                                collisionFramePosition=shift,
                            flags = pybullet.GEOM_FORCE_CONCAVE_TRIMESH,
                                                meshScale=meshScale)

        # self.plane =  pybullet.createMultiBody(baseMass=0,
        #                     baseInertialFramePosition=[0, 0, 0],
        #                     baseCollisionShapeIndex=collisionShapeId,
        #                     baseVisualShapeIndex=visualShapeId,
        #                     basePosition=[0,1,0],
        #                     useMaximalCoordinates=True)

        # self.plane =  pybullet.createMultiBody(baseMass=0,
        #                     baseInertialFramePosition=[0, 0, 0],
        #                     baseCollisionShapeIndex=collisionShapeId,
        #                     baseVisualShapeIndex=visualShapeId,
        #                     basePosition=[0,0,0],
        #                     useMaximalCoordinates=True)

        self.plane = self._pybullet_client.loadURDF("/home/jichen/multi-legged-rs/src/robot_envs/ground/plane_implicit.urdf")
 
        friction = 15
        collision_margin = 0.001
        pybullet.changeDynamics(self.plane, -1, lateralFriction=friction, collisionMargin=collision_margin)

        """" Traj Generator """
        # self.traj_gen = TrajectoryGenerator()
        # self.traj_gen.enable_pl = False

        """ Robot """
        self.robot.load_robot()
        # self.robot.load_robot()
        orientation = pybullet.getQuaternionFromEuler([0, 0, 0])
        orientation = pybullet.getQuaternionFromEuler([0, 0, 0])    
        pybullet.resetBasePositionAndOrientation(self.robot.robot_id,[0,0,0.2], orientation)  # start point1
        # pybullet.resetBasePositionAndOrientation(self.robot.robot_id,[0,4.1,1], orientation)  # start point3

        self.pose_offset = np.array([self.robot.leg_config['leg'+str(0)]['leg_pose'],
                                self.robot.leg_config['leg'+str(1)]['leg_pose'],
                                self.robot.leg_config['leg'+str(2)]['leg_pose'],
                                self.robot.leg_config['leg'+str(3)]['leg_pose'],
                                self.robot.leg_config['leg'+str(4)]['leg_pose'],
                                self.robot.leg_config['leg'+str(5)]['leg_pose']
                            ])

        self.x,self.y,self.z = np.zeros(6), np.zeros(6), np.zeros(6)
        self.vx,self.vy,self.vz = np.zeros(6), np.zeros(6), np.zeros(6)

        self.x_ref = np.zeros(6)
        self.y_ref = np.zeros(6)
        self.z_ref = np.zeros(6)
        self.current_poses = np.zeros([6,3])
        self.current_vel = np.zeros([6,3])
        self.last_pos = np.zeros([6,3])
        self.robot.reset_robot()

        self.touch_pub = rospy.Publisher('/touch_states',Float32MultiArray,queue_size=10)
        self.touch_msg = Float32MultiArray()

        self.force = np.zeros([3,6])
        self.force_f_old = np.zeros([3,6])

        

        self._pybullet_client.stepSimulation()

        self.force_vec = []
        self.force_vec_ori = []
        self.force_vec0 = []
        self.time_vec = []
        
        self.z_pos_ref = []
        self.x_pos_ref = []
        self.y_pos_ref = []
        self.phase_vec = np.zeros(6)

        self.v_vec = []
        self.y_vec = []
        self.z_vec = []
        self.v_ref_vec = []
        self.stance_force = 0

        self.error = []

        # self.signal_gen = GaitGenerator(cell_num=6, alpha=10, beta=10, mu=1, omega=0.5*np.pi, gamma=1)
        self.ad_force = 20

        self.avg_stance_f = 0

        """
        CSV files
        """
        f =  open("/home/jichen/hex_ctrl/src/data/ref_traj.csv", 'w')
        self.traj_writer = csv.writer(f)
        
        # f1 = open("/home/jichen/hex_ctrl/src/data/force_ad.csv", 'w')
        # self.force_writer = csv.writer(f1)

        f1 = open("/home/jichen/hex_ctrl/src/data/force_normal.csv", 'w')
        self.force_writer = csv.writer(f1)
        
    # def step(self, dt, duration, phase):
    def step(self, duration):
        # self.traj_gen.ad_duration = 5
        # force_f = np.zeros([3,6])
        # # print(phase)
            

            
        # # Apply Force to stance foots
        # cs = self.robot.get_contact_state(self.plane)


        # if duration<10:

        #     for i in range(6):
        #         if STANCEA < phase[i] <= STANCEB:    
        #             # print('a')
        #             self.robot.apply_adhesive_force(leg=i, force=self.ad_force)
        # else:
        #     for i in range(6):
        #         if STANCEA < phase[i] <= STANCEB and cs[i]==1: 
        #             # print('a')
        #             self.robot.apply_adhesive_force(leg=i, force=self.ad_force)


        # # Get force

        # for i in range(6):
        #     _, _, force = self.robot.get_single_leg_contact_state(self.plane,i, verbose=False)

        #     ff_old = self.force_f_old[:,i].reshape(3,1)

        #     ff_new = low_pass(force, ff_old, dt, 8)
        #     self.force_f_old[:,i] =  ff_new.ravel()    
        #     self.force[:,i] = force.ravel()
            



        

        # if duration > 5:
        #     self.time_vec.append(duration)
            
        #     for i in range(6):          

        #         # Get current foot positions and velocities
        #         pos = self.robot.foot_states_in_base_frame(i)  # link: EE_id -1
        #         vel = (pos - self.last_pos[i,:])/dt

        #         self.last_pos[i,:] = pos

        #         self.current_poses[i] = pos
        #         self.current_vel[i] = vel

        #         self.x[i], self.y[i], self.z[i] = pos[0]-self.pose_offset[i,0], pos[1]-self.pose_offset[i,1], pos[2]-self.pose_offset[i,2]
        #         self.vx[i], self.vy[i], self.vz[i] = vel[0], vel[1], vel[2]

        #     self.force_vec.append(self.force_f_old[2,1])
        #     self.force_vec0.append(self.force_f_old[2,0])
        #     self.force_vec_ori.append(self.force[2,1])
        #     self.force_writer.writerow([duration, self.force_f_old[2,1]])

        #     stance_g = self.traj_gen.get_stance_g(phase)
        #     self.avg_stance_f = get_stance_avg_f(stance_g,self.force_f_old[2,:])

        #     x_r, y_r, z_r, y_dot_ref, y1_ref, z1_ref, touchdown_f_leg = self.traj_gen.generate_control_signal_ag(self.x,self.y,self.z, self.vx, self.vy, self.vz, 
        #                                                                                 self.force_f_old, self.ad_force, dt, duration, phase, cs)

            
            
            
        #     # for leg in stance_g:

        #     print(stance_g)

        #     if len(touchdown_f_leg) != 0:
        #         for leg in touchdown_f_leg:
        #              self.robot.apply_adhesive_force(leg=leg, force=self.ad_force)

        #     self.traj_writer.writerow([y1_ref,z1_ref])
            

        #     self.x_ref = self.pose_offset[:,0] + x_r
        #     self.y_ref = self.pose_offset[:,1] + y_r
        #     self.z_ref = self.pose_offset[:,2] + z_r

        #     self.z_pos_ref.append(z_r[0])
        #     self.x_pos_ref.append(x_r[0])
        #     self.y_pos_ref.append(y_r[0])
            
        #     self.phase_vec = np.vstack((self.phase_vec, np.array(phase)))

        #     self.v_vec.append(self.vy[0])
        #     self.y_vec.append(self.y[0])
        #     self.v_ref_vec.append(y_dot_ref[0])
            

        #     # Execute Command
        #     for i in range(6):
        #         link_pos_ref = [self.x_ref[i],self.y_ref[i],self.z_ref[i]]
        #         leg_str = 'leg'+ str(i)
        #         dof_id = [6*i, 6*i+1, 6*i+2]
        #         joint_angles = self.robot.joint_angles_from_link_position(link_pos_ref,self.robot.leg_config[leg_str]["EE_id"]-1,dof_id)  # EE_id -1
        #         self.robot.set_joint_positions_by_leg(i,joint_angles)



        self._pybullet_client.stepSimulation()




    def reset(self):
        pass
