#!/usr/bin/env python
from typing import Sequence
import time
import numpy as np
from abb_egm_pyclient import EGMClient
from data.get_data import get_desJoints_L, get_desJoints_R, logic2abb


'''
Before running this script make sure that the starting pose of the robot (either real one or in RS) match the
starting pose of the computed joint angles. This script sends desired joint position to the robot. Note however, that
the IK does not take the real joint angles and velocities into account but supposes that they are applied without errors.
'''


UDP_PORT_LEFT = 6510
UDP_PORT_RIGHT = 6511
comp_conf_left = get_desJoints_L()
comp_conf_right = get_desJoints_R()
rate = 80

egm_client_left = EGMClient(port=UDP_PORT_LEFT)
egm_client_right = EGMClient(port=UDP_PORT_RIGHT)


for n in range(len(comp_conf_left[:,0])):

    # do stuff for the left arm
    robot_msg_L = egm_client_left.receive_msg()
    conf_L: Sequence[float] = robot_msg_L.feedBack.joints.joints
    joint7 = robot_msg_L.feedBack.externalJoints.joints[0]
    conf_L.insert(2, joint7)
    conf_yumi_L = np.array(conf_L)

    # do stuff for the right arm
    robot_msg_R = egm_client_right.receive_msg()
    conf_R: Sequence[float] = robot_msg_R.feedBack.joints.joints
    joint7 = robot_msg_R.feedBack.externalJoints.joints[0]
    conf_R.insert(2, joint7)
    conf_yumi_R = np.array(conf_R)

    print(f"Current configuration of left arm {conf_yumi_L}")
    print(f"Current configuration of right arm {conf_yumi_R}")

    # send out
    des_conf_L = np.degrees(comp_conf_left[n, :])
    des_conf_R = np.degrees(comp_conf_right[n, :])
    egm_client_left.send_planned_configuration(logic2abb(des_conf_L))
    egm_client_right.send_planned_configuration(logic2abb(des_conf_R))

    time.sleep(1/rate) 

# check if poses have been reached
n = 0
while n < 10:
    robot_msg = egm_client_left.receive_msg()
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