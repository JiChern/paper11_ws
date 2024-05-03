import numpy as np

import time
import pybullet_utils.bullet_client as bullet_client
import pybullet
import json
import typing
import rospy
import math

from scipy.signal import butter, lfilter, freqz




import matplotlib.pyplot as plt


ROBOT_URDF = "/home/jichen/paper11_ws/src/hexapod/robot/hexapod.urdf"
ROBOT_URDF_SIMP = "/home/jichen/hex_ctrl/src/robot_envs/hexapod/robot/hexapod_simp.urdf"
LEG_CONFIG = "/home/jichen/paper11_ws/src/hexapod/scripts/leg_config.json"
_IDENTITY_ORIENTATION = (0, 0, 0, 1)
  

class Hexapod(object):
    def __init__(self, pybullet_client):
        self._pybullet_client = pybullet_client
        self.robot_id = None

        self.debugline_id = 0

        with open(LEG_CONFIG) as json_file:
            self.leg_config = json.load(json_file)

        # self.sub_lists = []
        # for i in range(18):
        #     sub = message_filters.Subscriber()
           


    def load_robot(self):
        # self.robot_id = self._pybullet_client.loadURDF(ROBOT_URDF, basePosition=[0,0,0.2])
        orientation = pybullet.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = self._pybullet_client.loadURDF(ROBOT_URDF, basePosition=[0,0,0.1], baseOrientation=orientation)

    def load_robot_expC(self):
        # self.robot_id = self._pybullet_client.loadURDF(ROBOT_URDF, basePosition=[0,0,0.2])
        orientation = pybullet.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = self._pybullet_client.loadURDF(ROBOT_URDF_SIMP, basePosition=[0,0,0.1], baseOrientation=orientation)

    
    def get_base_pose(self):
        robot_pose, robot_ori = self._pybullet_client.getBasePositionAndOrientation(self.robot_id)
        return robot_pose, robot_ori


    def set_joint_positions_by_leg(self, leg, positions):
        # eg. set_joint_positions_by_leg(1,[0,0.1,0])
        leg_name = 'leg'+ str(leg)
        leg_motor_ids = self.leg_config[leg_name]['joint_ids']
        self._pybullet_client.setJointMotorControlArray(self.robot_id,jointIndices=leg_motor_ids,
                                                        controlMode=pybullet.POSITION_CONTROL, targetPositions=positions, 
                                                        positionGains = [1,1,1])
        maxForce = 0
        mode = pybullet.VELOCITY_CONTROL
        # ball_ids = self.leg_config[leg_name]['ball_ids']
        # for i in ball_ids:
        #     pybullet.setJointMotorControl2(1, i, controlMode=mode, force=maxForce)


    def get_leg_joint_states(self, leg):
        leg_name = 'leg'+ str(leg)
        leg_motor_ids = self.leg_config[leg_name]['joint_ids']
        js = self._pybullet_client.getJointStates(self.robot_id,leg_motor_ids)
        motor_positions = [state[0] for state in js]
        motor_velocities = [state[1] for state in js]
        return leg_motor_ids, motor_positions, motor_velocities



    def foot_pose_in_base_frame(self, leg):
        """Computes the link's local position in the robot frame.

        Args:
            robot: A robot instance.
            link_id: The link to calculate its relative position.

        Returns:
            The relative position of the link.
        """
        leg_name = 'leg'+ str(leg)
        link_id = self.leg_config[leg_name]['EE_id']
        base_position, base_orientation = self.get_base_pose()
        inverse_translation, inverse_rotation = self._pybullet_client.invertTransform(
            base_position, base_orientation)

        link_state = self._pybullet_client.getLinkState(self.robot_id, link_id, computeLinkVelocity=True, computeForwardKinematics=True)
        link_position = link_state[0]
        link_vel = link_state[6]
        link_local_position, _ = self._pybullet_client.multiplyTransforms(
            inverse_translation, inverse_rotation, link_position, (0, 0, 0, 1))
        link_local_vel, _ = self._pybullet_client.multiplyTransforms(
            inverse_translation, inverse_rotation, link_vel, (0, 0, 0, 1))


        return np.array(link_local_position), np.array(link_local_vel)  


    def foot_states_in_base_frame(self,leg):
        leg_name = 'leg'+ str(leg)
        link_id = self.leg_config[leg_name]['EE_id']
        base_position, base_orientation = self.get_base_pose()
        inverse_translation, inverse_rotation = self._pybullet_client.invertTransform(
            base_position, base_orientation)

        # info = pybullet.getJointInfo(self.robot_id,0)
        # print(info)


        link_state = self._pybullet_client.getLinkState(self.robot_id, link_id,computeLinkVelocity=False, computeForwardKinematics=True)

        link_position = link_state[0]


        link_local_position, _ = self._pybullet_client.multiplyTransforms(
            inverse_translation, inverse_rotation, link_position, (0, 0, 0, 1))


        return np.array(link_local_position)
    

    def joint_angles_from_link_position(self,
        link_position: typing.Sequence[float],
        link_id: int,
        joint_ids: typing.Sequence[int],
        base_translation: typing.Sequence[float] = (0, 0, 0),
        base_rotation: typing.Sequence[float] = (0, 0, 0, 1)):
        """Uses Inverse Kinematics to calculate joint angles.

        Args:
            robot: A robot instance.
            link_position: The (x, y, z) of the link in the body frame. This local frame
            is transformed relative to the COM frame using a given translation and
            rotation.
            link_id: The link id as returned from loadURDF.
            joint_ids: The positional index of the joints. This can be different from
            the joint unique ids.
            base_translation: Additional base translation.
            base_rotation: Additional base rotation.

        Returns:
            A list of joint angles.
        """
        # Projects to local frame.
        base_position, base_orientation = self.get_base_pose()
        base_position, base_orientation = self._pybullet_client.multiplyTransforms(
            base_position, base_orientation, base_translation, base_rotation)

        # Projects to world space.
        world_link_pos, _ = self._pybullet_client.multiplyTransforms(
            base_position, base_orientation, link_position, _IDENTITY_ORIENTATION)
        ik_solver = 0
        all_joint_angles = self._pybullet_client.calculateInverseKinematics(
            self.robot_id, link_id, world_link_pos, solver=ik_solver)

        # print(len(all_joint_angles))
        # print(joint_ids)

        # Extract the relevant joint angles.
        joint_angles = [all_joint_angles[i] for i in joint_ids]
        return joint_angles

    def get_leg_dof_id(self, leg_id):
        dof_id = [leg_id*3,leg_id*3+1,leg_id*3+2]
        return dof_id

    def reset_robot(self):
        desired_foot_poses = np.zeros([6,3])
        current_foot_poses = np.zeros([6,3])
        for i in range(6):
            leg_str = 'leg'+ str(i)
            desired_foot_poses[i,:] = np.array(self.leg_config[leg_str]['leg_pose'])
            x = self.foot_states_in_base_frame(i)
            current_foot_poses[i,:] = x

        print('desired_fp: ', desired_foot_poses)
        print('current_fp: ', current_foot_poses)
        
        error = np.sqrt(np.sum(np.square(desired_foot_poses - current_foot_poses)))
        thres = 0.1e-2
        
        while not rospy.is_shutdown():
            for i in range(6):
                # dof_id = [6*i, 6*i+1, 6*i+2]
                dof_id = [3*i, 3*i+1, 3*i+2]
                leg_str = 'leg'+ str(i)
                joint_angles = self.joint_angles_from_link_position(desired_foot_poses[i],self.leg_config[leg_str]["EE_id"],dof_id)
                self.set_joint_positions_by_leg(i,joint_angles)

                x = self.foot_states_in_base_frame(i)
                current_foot_poses[i,:] = x
            self._pybullet_client.stepSimulation()
            error = np.sqrt(np.sum(np.square(desired_foot_poses - current_foot_poses)))

            if error < thres:
                print('aaa')
                break

            print('error: ',error)
        

    def apply_adhesive_force(self, leg, force):
        # check contacts of adhesive pads
        leg_str = "leg" + str(leg)
        ee_id = self.leg_config[leg_str]["EE_id"]
        self._pybullet_client.applyExternalForce(self.robot_id,ee_id,[0.0,0.0,force],[0.0,0.0,0.0],pybullet.LINK_FRAME)

    def apply_force(self, leg, force):
        # check contacts of adhesive pads
        leg_str = "leg" + str(leg)
        ee_id = self.leg_config[leg_str]["EE_id"]
        self._pybullet_client.applyExternalForce(self.robot_id,ee_id,force,[0.0,0.0,0.0],pybullet.LINK_FRAME)
        
    def get_contact_state(self, ground):
        contact_vec = np.zeros(6)
        for i in range(6):
            leg_str = "leg"+str(i)
            ee_id = self.leg_config[leg_str]["EE_id"]
            list_c = pybullet.getContactPoints(bodyA=self.robot_id, bodyB=ground,linkIndexA=ee_id-1)
            force = 0
            if len(list_c) == 0:
                contact_vec[i] = 0
            else:
                for element in list_c:
                    ptForce = element[9]
                    force = force + ptForce


                # print(force)
                contact_vec[i] = 1
    
            
        return contact_vec

    def get_single_leg_contact_state(self, ground, leg, verbose=True):
        contact = 0

        leg_str = "leg"+str(leg)
        ee_id = self.leg_config[leg_str]["EE_id"]
        # print(ee_id)
        list_c = pybullet.getContactPoints(bodyA=self.robot_id, bodyB=ground,linkIndexA=ee_id-1)
        force = 0
        friction_force1 = 0
        friction_force2 = 0

        pos = np.zeros(3)
        # print(list_c)
        
        if len(list_c) != 0:
            for element in list_c:
                ptForce = element[9]
                pt_fric1 = element[10]
                pt_fric2 = element[12]
                posi = np.array(element[5])
                
                force = force + ptForce
                friction_force1 = friction_force1 + pt_fric1
                friction_force2 = friction_force2 + pt_fric2

                pos = np.vstack((pos,posi))

                

            # Debuglines
            if leg == 0 and verbose:
                pos = np.delete(pos,0, axis=0)
                mean_pos = np.mean(pos, axis=0)

                k = 0.01
                end_pt = [mean_pos[0]+k*friction_force2, mean_pos[1]-k*friction_force1, mean_pos[2]+k*force]
                self.debugline_id = pybullet.addUserDebugLine(mean_pos, end_pt, [1, 0, 0], 2.0, replaceItemUniqueId=self.debugline_id)

            # print(force)

        if force!=0 and friction_force1!=0 and friction_force2!=0:
            contact = 1
        
        force_w = np.array([friction_force2,-friction_force1,force]).reshape(3,1)

        base_position, base_orientation = self.get_base_pose()
        inverse_translation, inverse_rotation = self._pybullet_client.invertTransform(
            base_position, base_orientation)

        # force_b, _ = self._pybullet_client.multiplyTransforms(
        #     inverse_translation, inverse_rotation, force_w, (0, 0, 0, 1))
        force_b, _ = self._pybullet_client.multiplyTransforms(
            np.zeros(3), inverse_rotation, force_w, (0, 0, 0, 1))
        
        force_b = np.array(force_b).reshape(3,1)
            
        return contact, force_w, force_b

    def compute_jacobian_by_legs(self, leg):
        """Computes the Jacobian matrix for the given link.

        Args:
            robot: A robot instance.
            link_id: The link id as returned from loadURDF.

        Returns:
            The 3 x N transposed Jacobian matrix. where N is the total DoFs of the
            robot. For a quadruped, the first 6 columns of the matrix corresponds to
            the CoM translation and rotation. The columns corresponds to a leg can be
            extracted with indices [6 + leg_id * 3: 6 + leg_id * 3 + 3].

        """
        leg_str = "leg"+str(leg)
        ee_id = self.leg_config[leg_str]["EE_id"] - 1
        dof_list = self.leg_config['full_body']['DoF_list']
        joint_states = self._pybullet_client.getJointStates(
        self.robot_id, dof_list)
        all_joint_angles = [state[0] for state in joint_states]

        zero_vec = [0] * len(all_joint_angles)

        jv, _ = self._pybullet_client.calculateJacobian(self.robot_id, ee_id,
                                                        (0, 0, 0), all_joint_angles,
                                                        zero_vec, zero_vec)
        jacobian = np.array(jv)
        leg_jacobian = jacobian[:,6+leg*6:6+leg*6+3]
        
        return leg_jacobian


    def Admittance(self, force_b, x_d, v_d, v_d_dot ,x_now, v_now, dt, ad_force):
        """
        Admittance Property: M*x_e_ddot + B*x_e_dot + K*x_e = Fe
        ,where Fe is the external force measured,

        """
        M = 0.5
        K = np.diag([2000,2000,2000])
        sigma = 1 # >1 过阻尼， <1 欠阻尼
        B = 2*sigma*np.sqrt(M*K)


        # print(K)

        x_e = x_now - x_d - (-v_d)*dt
        x_e_dot = v_now - v_d - (-v_d_dot)*dt 

        force_b[2] = np.clip(force_b[2], None, ad_force)

        F_e = force_b - np.array([0,0,ad_force]).reshape(3,1)

        if F_e[2] > -2:
            F_e[2] = 0

        F_e[2] = np.clip(F_e[2],-ad_force,ad_force)



        f_thresxy = 0.1
        f_thresz = -5

        # print('K', K)

        if F_e[2] < f_thresz:
            K = np.diag([500,500,500])
            if F_e[2] < -ad_force/2:
                K = np.diag([250,250,250])
        
        

        # x_e_ddot_r = (1/M)*(F_e + B*x_e_dot + K*x_e)

        x_e_ddot_r = (1/M)*(F_e - B@x_e_dot - K@x_e)



        x_e_dot_r = x_e_dot + x_e_ddot_r *dt 
        x_e_r = x_e + x_e_dot_r * dt
        x_r = x_d + x_e_r


        return x_r, F_e, x_e_ddot_r, x_e_r, x_e      
    


