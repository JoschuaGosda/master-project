#!/usr/bin/env python
from typing import Sequence
import time
import copy
import numpy as np
from abb_egm_pyclient import EGMClient
import libs.invKin as invKin
from data.get_data import get_trajectory, transform2yumi_workspace, place_trajectory, logic2abb

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

yumi_right = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_right.urdf")
yumi_left = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_left.urdf")


jointVelocities_L = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
jointVelocities_R = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])



for index, (pos1, vel1, pos2, vel2, phi, phi_dot) in enumerate(zip(p1, v1, p2, v2, phi_delta, dphi)):
    tstart = time.time()
    t0 = time.time()
    # get the current joints angles for the left arm
    
    robot_msg_L = egm_client_L.receive_msg()
    conf_L: Sequence[float] = robot_msg_L.feedBack.joints.joints
    joint7 = robot_msg_L.feedBack.externalJoints.joints[0]
    conf_L.insert(2, joint7)
    jointAngles_L = np.radians(np.array(conf_L))

    # get the current joints angles for the right arm
    robot_msg_R = egm_client_R.receive_msg()
    conf_R: Sequence[float] = robot_msg_R.feedBack.joints.joints
    joint7 = robot_msg_R.feedBack.externalJoints.joints[0]
    conf_R.insert(2, joint7)
    jointAngles_R = np.radians(np.array(conf_R))



    t1 = time.time()

    # compute the resulting jointVelocities
    if index > 0:
        jointVelocities_L = (jointAngles_L - jointAngles_L_old)/rate
        jointVelocities_R = (jointAngles_R - jointAngles_R_old)/rate

    desPose_L = np.concatenate((pos1, phi), axis=0) 
    desVelocities_L = np.concatenate((vel1, phi_dot), axis=0) 

    desPose_R = np.concatenate((pos2, phi), axis=0) 
    desVelocities_R = np.concatenate((vel2, phi_dot), axis=0) 

    t2 = time.time()

    yumi_left.set_jointValues(jointAngles_L, jointVelocities_L)
    yumi_left.set_desPoseVel(desPose_L, desVelocities_L)
    yumi_left.process()

    yumi_right.set_jointValues(jointAngles_R, jointVelocities_R)
    yumi_right.set_desPoseVel(desPose_R, desVelocities_R)
    yumi_right.process()

    ik_jointAngles_L = yumi_left.get_newJointValues() # computed joint values from IK

    ik_jointAngles_R = yumi_right.get_newJointValues() # computed joint values from IK

    t3 = time.time()

    #print(f"Current configuration of left arm {conf_yumi_L}")
    #print(f"Current configuration of right arm {conf_yumi_R}")

    # transform to degrees as egm wants it
    des_conf_L = np.degrees(ik_jointAngles_L)
    des_conf_R = np.degrees(ik_jointAngles_R)

    egm_client_L.send_planned_configuration(logic2abb(des_conf_L))
    egm_client_R.send_planned_configuration(logic2abb(des_conf_R))

    t4 = time.time()

    # save joint values to compute joint velocities in the next iteration
    jointAngles_L_old = copy.copy(jointAngles_L)
    jointAngles_R_old = copy.copy(jointAngles_R)

    tend = time.time()

    time.sleep(1/rate) 


print("time per iteration:  ", (tend-tstart)/len(p1[:,0]))
print("time to receive msg from RS: ", t1-t0)
print("time to build desPose & desVel:  ", t2-t1)
print("time to perform ik:  ", t3-t2)
print("time to send config to RS:   ", t4-t3)

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
