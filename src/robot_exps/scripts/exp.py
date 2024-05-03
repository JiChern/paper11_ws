from re import T
from env import HexEnv
import rospy
import time
import matplotlib.pyplot as plt
import numpy as np
import pybullet
import math

import pandas as pd

if __name__ == "__main__":
    rospy.init_node('env_test')



    """ 6 cells """
    pi = math.pi
    tpi = math.pi*2
    cater = tpi/3*np.ones(6)
    tri = pi*np.ones(6)
    metach = tpi/6*np.ones(6)
    wave = np.array([tpi/3,tpi/3, pi, tpi/3,tpi/3, pi/3])
    test_theta = tpi*np.ones(6)



    env = HexEnv(gravity=-9.8)

    

    start_time = time.time()
    duration = 0
    
    dt = 0.01
    duration = 0
    start_time = time.time()



    last_pl = True
    r = rospy.Rate(200)

    # env.traj_gen.enable_pl = True

    ad_force = 20
    env.ad_force = ad_force

    # env.traj_gen.ad_duration=5

    # Initial forces
    beta = 0.999

    # env.signal_gen.set_theta(tri)
    # env.signal_gen.start_oscillate_soft(1000)

    phase_vec = np.zeros(6)
    force_vec = np.zeros(6)

    stance_f_vec = []


    # while duration < 2:
    #     duration = time.time() - start_time
    #     for i in range(6):
    #             env.robot.apply_adhesive_force(leg=i, force=ad_force)
    #     env._pybullet_client.stepSimulation()

    while not rospy.is_shutdown():

        loop_start_time = time.time()

        # print(duration)

        # if duration>5:
        #     pybullet.setGravity(0.0,0.0,0.0)

        # if  15 < duration < 50:
            
        #     target_theta = cater
        #     new_theta = beta*env.signal_gen.theta + (1-beta)*target_theta
        #     # print(new_theta)
        #     env.signal_gen.set_theta(new_theta)

        # elif 50 <= duration < 110:
        #     target_theta = metach
        #     new_theta = beta*env.signal_gen.theta + (1-beta)*target_theta
        #     # print(new_theta)
        #     env.signal_gen.set_theta(new_theta)
        # else:
        #     pass

        # if duration >= 110:
        #     break

        # if not env.traj_gen.phase_lock:    
            
        #     r_phase = env.signal_gen.update_soft(steps=dt)

        #     phase_now = np.array(r_phase)
        #     phase_vec = np.vstack((phase_vec, phase_now))

        #     stance_f_vec.append(env.avg_stance_f)
            



        # # Phase lock state
        # if env.traj_gen.phase_lock != last_pl:
        #     print('phase lock: ', env.traj_gen.phase_lock)
        #     last_pl = env.traj_gen.phase_lock

        # info = pybullet.getJointInfo(env.robot.robot_id,10)
        # info1 = pybullet.getJointInfo(env.robot.robot_id,10)
        # info2 = pybullet.getJointInfo(env.robot.robot_id,10)

        # print(info)
        # print(info1)
        # print(info2)

        env.step(duration)
        r.sleep()

        
        # dt = time.time()-loop_start_time
        duration = time.time()-start_time
        # print(dt)

        # if duration > 20:
        #     break


    pd.DataFrame(stance_f_vec).to_csv("data/expC_force.csv", header=None, index=None)
    pd.DataFrame(phase_vec).to_csv("data/expC_phases.csv", header=None, index=None)



    # plt.figure()
    # # plt.plot(env.time_vec,env.force_vec_ori)
    # plt.plot(env.time_vec,env.force_vec)

    plt.figure()
    # phase_vec = env.phase_vec[1:]
    phase_vec = phase_vec[1:]
    plt.plot(phase_vec[:,0])
    plt.plot(phase_vec[:,1])
    plt.plot(phase_vec[:,2])
    plt.plot(phase_vec[:,3])
    plt.plot(phase_vec[:,4])
    plt.plot(phase_vec[:,5])
    

    # plt.plot(env.time_vec, env.y_pos_ref, label='y_ref')
    # plt.plot(env.traj_gen.z_ref_vec, label='z traj')
    # plt.plot(env.traj_gen.y_ref_vec, label='y traj')
    # plt.plot(np.array(env.traj_gen.phase_vec)/10, label='phase1')
    # plt.plot(env.time_vec, env.phase_vec/100)
    # plt.plot(env.time_vec,env.force_vec0)
    # plt.plot(env.time_vec,0.1*np.array(env.phase_vec), label='phase')
    # vec_len = len(env.time_vec)
    # plt.plot(env.time_vec,0.1*0.5*np.ones(vec_len))
    # plt.plot(env.time_vec, env.v_vec, label='vy')
    # plt.plot(env.time_vec, env.y_vec, label='y')
    # plt.plot(env.time_vec, env.v_ref_vec, label='y dot')
    # plt.legend()
    
    plt.grid()

    plt.figure()
    plt.plot(stance_f_vec)


    plt.show()
