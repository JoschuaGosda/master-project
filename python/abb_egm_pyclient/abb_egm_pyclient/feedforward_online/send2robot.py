#!/usr/bin/env python
from typing import Sequence
import time
import copy
import numpy as np
from abb_egm_pyclient import EGMClient
from data.get_data import get_desJoints_L, get_desJoints_R
from libs.invKin import gpm
from libs.invKin import loadKinematicModel
from data.get_data import get_trajectory, transform2yumi_workspace, place_trajectory, Yumi, logic2abb

'''
Before running this script make sure that the starting pose of the robot (either real one or in RS) match the
starting pose of the computed joint angles. This script sends desired joint position to the robot. It takes to real
joint positions at the robot into account
'''

# READ IN THE TRAJECTORY
# get the data in the yumi workspace
p1, v1, p2, v2, phi_delta, dphi = get_trajectory()
p1, v1, p2, v2, phi_delta, dphi = transform2yumi_workspace(p1, v1, p2, v2, phi_delta, dphi)

# define staring postition in workspace for left arm - found by try and error in RS
p1_start_des = np.array([0.3, 0.2, 0.2])
p1, p2 = place_trajectory(p1_start_des, p1, p2)

# setup for UDP connection
UDP_PORT_LEFT = 6510
UDP_PORT_RIGHT = 6511
rate = 80

egm_client_L = EGMClient(port=UDP_PORT_LEFT)
egm_client_R = EGMClient(port=UDP_PORT_RIGHT)

kinematic_model_ptr_L = loadKinematicModel("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_left.urdf")
kinematic_model_ptr_R = loadKinematicModel("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_right.urdf")

jointVelocities_L = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
jointVelocities_R = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

for index, (pos1, vel1, pos2, vel2, phi, phi_dot) in enumerate(zip(p1, v1, p2, v2, phi_delta, dphi)):
    
    t0 = time.time()

    # get the current joints angles for the left arm
    robot_msg_L = egm_client_L.receive_msg()
    conf_L: Sequence[float] = robot_msg_L.feedBack.joints.joints
    joint7 = robot_msg_L.feedBack.externalJoints.joints[0]
    conf_L.insert(2, joint7)
    jointAngles_L = np.radians(np.array(conf_L))

    t1 = time.time()
    ex_time = t1-t0
    print(f'computation time for msg left: {ex_time}')

    # get the current joints angles for the right arm
    robot_msg_R = egm_client_R.receive_msg()
    conf_R: Sequence[float] = robot_msg_R.feedBack.joints.joints
    joint7 = robot_msg_R.feedBack.externalJoints.joints[0]
    conf_R.insert(2, joint7)
    jointAngles_R = np.radians(np.array(conf_R))

    t2 = time.time()
    ex_time = t2-t1
    print(f'computation time for msg right: {ex_time}')

    # compute the resulting jointVelocities
    if index > 0:
        jointVelocities_L = (jointAngles_L - jointAngles_L_old)/rate
        jointVelocities_R = (jointAngles_R - jointAngles_R_old)/rate

    desPose_L = np.concatenate((pos1, phi), axis=0) 
    desVelocities_L = np.concatenate((vel1, phi_dot), axis=0) 

    desPose_R = np.concatenate((pos2, phi), axis=0) 
    desVelocities_R = np.concatenate((vel2, phi_dot), axis=0) 

    t3 = time.time()
    ex_time = t3-t2
    print(f'computation time for assembling arrays: {ex_time}')

    ik_jointAngles_L, ik_pose_L = gpm(desPose_L, desVelocities_L, jointAngles_L, jointVelocities_L, kinematic_model_ptr_L)
    ik_jointAngles_R, ik_pose_R = gpm(desPose_R, desVelocities_R, jointAngles_R, jointVelocities_R, kinematic_model_ptr_R)

    t4 = time.time()
    ex_time = t4-t3
    print(f'computation time for ik: {ex_time}')

    #print(f"Current configuration of left arm {conf_yumi_L}")
    #print(f"Current configuration of right arm {conf_yumi_R}")

    # transform to degrees as egm wants it
    des_conf_L = np.degrees(ik_jointAngles_L)
    des_conf_R = np.degrees(ik_jointAngles_R)

    egm_client_L.send_planned_configuration(logic2abb(des_conf_L))
    egm_client_R.send_planned_configuration(logic2abb(des_conf_R))

    t5 = time.time()
    ex_time = t5-t4
    print(f'computation time for sending out joints: {ex_time}')

    # save joint values to compute joint velocities in the next iteration
    jointAngles_L_old = copy.copy(jointAngles_L)
    jointAngles_R_old = copy.copy(jointAngles_R)

    t6 = time.time()
    ex_time = t6-t0
    print(f'computation time whole loop: {ex_time}')
    time.sleep(1/rate) 

# check if poses have been reached
n = 0
while n < 10:
    robot_msg = egm_client_L.receive_msg()
    if robot_msg.mciConvergenceMet:
        print("Joint positions converged.")
        break
    else:
        print("Waiting for convergence.")
    n += 1
    time.sleep(1)
else:
    raise TimeoutError(f"Joint positions did not converge after {n} s timeout.")






# TODO: check if all devices and neccessary files are online/in the right place
# show if thats the case and ask user if he wants to start control 

# TODO: read the data from g-code and transform it into the global frame of yumi
# make sure to set the first configuration as the fine point in robot studio, so that they
# virtual and real configuration are synchronized

# TODO: loop over the g-code positions (check g-code of its based on constant speed)
# instructions in G code are G1 instruction meaning moving at a constant feed (velocity in axis direction)
# https://www.seniorcare2share.com/how-does-g-code-work/

    # TODO: read current configuration via egm client

    # TODO: use RL to compute the current jacobian

    # TODO: call the C++ function to get the joint values
    #   input:  current jacobian, nullspace gradient, desired position in task space, sample time 

    # TODO: send to obtained joint values via egm client