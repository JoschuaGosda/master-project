#!/usr/bin/env python
import sys
from typing import Sequence
import time

import numpy as np

from abb_egm_pyclient import EGMClient

'''
The example of the repo https://gitlab.control.lth.se/tetov/abb_egm_pyclient is a good starting point but
code needs to be adapted to projects goal of foam cutting. Structure of functions needs to changed since
further steps like computing the IK needs to be performed between receiving and sending positions and 
reading force sensor data.

'''


DEFAULT_UDP_PORT = 6510
comp_conf = np.load('./abb_egm_pyclient/abb_egm_pyclient/desJointAngles_left.npy')
rate = 80

def logic2abb(joint_values):
    return joint_values[[0, 1, 3, 4, 5, 6, 2]]

def abb2logic(joint_values):
    return joint_values[[0, 1, 5, 2, 3, 4]]



#mylist = [1, 2, 3, 4, 5, 6]
#mylist.insert(2, 7)
#print("mylist", mylist)

egm_client = EGMClient(port=DEFAULT_UDP_PORT)

# get the current values of joints
robot_msg = egm_client.receive_msg()
conf: Sequence[float] = robot_msg.feedBack.joints.joints # get ABB's standard six joint values
joint7 = robot_msg.feedBack.externalJoints.joints[0]
conf.insert(2, joint7)
conf_yumi = np.array(conf)


conf_des = comp_conf[0, :]
print("des conf", conf_des)
print("arranged", logic2abb(conf_des))

print(conf_yumi)
print(conf_des)


# check if real joint value match the computed starting joint positions 
#conf_diff = np.abs(logic2abb(conf_des) - conf_yumi)
#condition = np.array([0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03]) # equals 2 degrees
#assert (conf_diff-condition).all(), 'Starting configurations are not synchronized'


for n in range(len(comp_conf[:,0])):
    sTime = time.time()
    robot_msg = egm_client.receive_msg()
    conf: Sequence[float] = robot_msg.feedBack.joints.joints
    joint7 = robot_msg.feedBack.externalJoints.joints[0]
    conf.insert(2, joint7)
    conf_yumi = np.array(conf)

    print(f"Current configuration {conf_yumi}")

    des_conf = np.degrees(comp_conf[n, :])
    egm_client.send_planned_configuration(logic2abb(des_conf))

    # take the execution time into account that loops stays iterating with the rate frequency
    # get more important the higher rate becomes like 100-250
    sleeping_time = 1/rate - (time.time()-sTime)
    time.sleep(sleeping_time) 

# after all messages are sent out, wait for 10 sec and check if positions converged
n = 0
while n < 10:
    robot_msg = egm_client.receive_msg()
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