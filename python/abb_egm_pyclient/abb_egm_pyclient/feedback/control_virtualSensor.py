# TODO: check if all devices and neccessary files are online/in the right place
# show if thats the case and ask user if he wants to start control 

#!/usr/bin/env python
from typing import Sequence
import time
import copy
import numpy as np
from abb_egm_pyclient import EGMClient
import libs.invKin as invKin
from data.get_data import get_trajectory, transform2yumi_workspace, place_trajectory, logic2abb
from matplotlib import pyplot as plt

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
UDP_PORT_RIGHT = 6511
rate = 80

# take the first desired taskSpaceInput for the right arm and simulate force sensor data 
trans_const = copy.copy(p2[0, :])
#rot_const = copy.copy(phi_delta[0, :])
rot_const = np.array([np.pi/2, 0, 0])
desPose_R_const = np.concatenate((trans_const, rot_const), axis=0) 


# generate sinusoidal test data for the virtual force sensor
# sin(t/pi *4) + 2, test function, test for 5 seconds
numDataPoints = round(5/(1/rate))
forceInput = np.zeros((numDataPoints))
for i in range(numDataPoints):
    forceInput[i] = np.sin(i/np.pi * 1/rate * 4)+2

m_time = np.linspace(0, 5, num=numDataPoints)

""" fig = plt.figure()
plt.plot(m_time, forceInput, label='desired test signal') #
fig.get_axes()[0].set_xlabel('time')
fig.get_axes()[0].set_ylabel('virtual force signal')
plt.legend()
plt.title('test signal')
plt.show() """

egm_client_R = EGMClient(port=UDP_PORT_RIGHT)

yumi_right = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_right.urdf")

desVelocities_R_const = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
jointVelocities_R = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

for index, force in enumerate(forceInput):

    # get the current joints angles for the right arm
    robot_msg_R = egm_client_R.receive_msg()
    conf_R: Sequence[float] = robot_msg_R.feedBack.joints.joints
    joint7 = robot_msg_R.feedBack.externalJoints.joints[0]
    conf_R.insert(2, joint7)
    jointAngles_R = np.radians(np.array(conf_R))

    # compute the resulting jointVelocities
    if index > 0:
        jointVelocities_R = (jointAngles_R - jointAngles_R_old)/rate


    yumi_right.set_jointValues(jointAngles_R, jointVelocities_R)
    yumi_right.set_desPoseVel(desPose_R_const, desVelocities_R_const)
    yumi_right.set_force(force)
    yumi_right.process()


    ik_jointAngles_R = yumi_right.get_newJointValues() # computed joint values from IK


    #print(f"Current configuration of left arm {conf_yumi_L}")
    #print(f"Current configuration of right arm {conf_yumi_R}")

    # transform to degrees as egm wants it
    des_conf_R = np.degrees(ik_jointAngles_R)

    egm_client_R.send_planned_configuration(logic2abb(des_conf_R))


    # save joint values to compute joint velocities in the next iteration
    jointAngles_R_old = copy.copy(jointAngles_R)


    time.sleep(1/rate) 



# check if poses have been reached
n = 0
while n < 10:
    robot_msg = egm_client_R.receive_msg()
    if robot_msg.mciConvergenceMet:
        print("Joint positions converged.")
        break
    else:
        print("Waiting for convergence.")
    n += 1
    time.sleep(1)
else:
    raise TimeoutError(f"Joint positions did not converge after {n} s timeout.")
