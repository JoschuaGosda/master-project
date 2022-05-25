# TODO: check if all devices and neccessary files are online/in the right place
# show if thats the case and ask user if he wants to start control 

#!/usr/bin/env python
from ssl import PROTOCOL_TLSv1_1
from typing import Sequence
import time
import copy
import numpy as np
from abb_egm_pyclient import EGMClient
import libs.invKin as invKin
from data.get_data import get_trajectory, transform2yumi_workspace, place_trajectory, logic2abb
from matplotlib import pyplot as plt
import serial.tools.list_ports

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

# take the first desired taskSpaceInput for the right arm and simulate force sensor data 
translation_R_start = copy.copy(p2[0, :])
#rotation_R_start = copy.copy(phi_delta[0, :])
rotation_R_start = np.array([0, 0, 0])
desPose_R_start = np.concatenate((translation_R_start, rotation_R_start), axis=0) 

egm_client_R = EGMClient(port=UDP_PORT_RIGHT)
egm_client_L = EGMClient(port=UDP_PORT_LEFT)

yumi_right = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_right.urdf")
yumi_right.set_operationPoint(0.7)
yumi_right.set_kp(0.2)
yumi_right.set_hybridControl(True)

yumi_left = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_left.urdf")

desVelocities_R_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
jointVelocities_R = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# SETUP SERIAL INTERFACE
# find active ports
ports = serial.tools.list_ports.comports()
arduino = serial.Serial()

myPort = '/dev/ttyACM0'
myBaudRate = 38400
portList = []

print("AVAILABLE PORTS \n")
for onePort in ports:
    portList.append(str(onePort))
    print(str(onePort))

# setup configuration for serial interface
arduino.baudrate = myBaudRate
arduino.port = myPort
arduino.open()

time.sleep(2)
arduino.flushInput()

# wait until first serial data from arduino is available
while not arduino.in_waiting:
    pass
    
i = 0
force = 0.0 # 0.5 [N] seems to be a good value for the control
timestamp = time.time()
cutting = False
traj_samples = len(p1[:, 0]) 

print("\n Force control only to tension the wire...")

while True and arduino.isOpen():

    # check for new force data
    if arduino.in_waiting: # get the number of bytes in the input buffer
        packet = arduino.readline() # type: bytes  
        str_receive = packet.decode('utf-8').rstrip('\n')
        force = float(str_receive)/1000.0 # mN to Newton

    # force control only until wire is tensioned
    if not cutting:
        if (time.time() - timestamp) >= (1.0/rate):
            # get the current joints angles for the right arm
            robot_msg_R = egm_client_R.receive_msg()
            conf_R: Sequence[float] = robot_msg_R.feedBack.joints.joints
            joint7 = robot_msg_R.feedBack.externalJoints.joints[0]
            conf_R.insert(2, joint7)
            jointAngles_R = np.radians(np.array(conf_R))

            # compute the resulting jointVelocities
            if i > 0:
                jointVelocities_R = (jointAngles_R - jointAngles_R_old)/rate

            yumi_right.set_jointValues(jointAngles_R, jointVelocities_R)
            yumi_right.set_desPoseVel(desPose_R_start, desVelocities_R_start)
            yumi_right.set_force(force)
            yumi_right.process()

            print("force in [N] is: %5.2f" %  force)

            ik_jointAngles_R = yumi_right.get_newJointValues() # computed joint values from IK

            # transform to degrees as egm wants it
            des_conf_R = np.degrees(ik_jointAngles_R)

            egm_client_R.send_planned_configuration(logic2abb(des_conf_R))

            # save joint values to compute joint velocities in the next iteration
            jointAngles_R_old = copy.copy(jointAngles_R)

            i = i+1
            timestamp = time.time()
            if (force > 0.7):
                cutting = True
                jointVelocities_L = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                jointVelocities_R = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                i = 0 # reset counter
                print("Changing to hybrid control now...")
    
    # hybrid control - cutting is True
    else:
        if (time.time() - timestamp) >= (1.0/rate):
            # copy array entries to local variables
            pos1 = p1[i, :]
            vel1 = v1[i, :]
            pos2 = p2[i, :]
            vel2 = v2[i, :]
            phi = phi_delta[i, :]
            phi_dot = dphi[i, :]

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

            # compute the resulting jointVelocities
            if i > 0:
                jointVelocities_L = (jointAngles_L - jointAngles_L_old)/rate
                jointVelocities_R = (jointAngles_R - jointAngles_R_old)/rate

            desPose_L = np.concatenate((pos1, phi), axis=0) 
            desVelocities_L = np.concatenate((vel1, phi_dot), axis=0) 

            desPose_R = np.concatenate((pos2, phi), axis=0) 
            desVelocities_R = np.concatenate((vel2, phi_dot), axis=0) 

            yumi_left.set_jointValues(jointAngles_L, jointVelocities_L)
            yumi_left.set_desPoseVel(desPose_L, desVelocities_L)
            yumi_left.process()

            yumi_right.set_jointValues(jointAngles_R, jointVelocities_R)
            yumi_right.set_desPoseVel(desPose_R, desVelocities_R)
            yumi_right.set_force(force)
            yumi_right.process()

            ik_jointAngles_L = yumi_left.get_newJointValues() # computed joint values from IK
            ik_jointAngles_R = yumi_right.get_newJointValues() # computed joint values from IK

            # transform to degrees as egm wants it
            des_conf_L = np.degrees(ik_jointAngles_L)
            des_conf_R = np.degrees(ik_jointAngles_R)

            egm_client_L.send_planned_configuration(logic2abb(des_conf_L))
            egm_client_R.send_planned_configuration(logic2abb(des_conf_R))

            # save joint values to compute joint velocities in the next iteration
            jointAngles_L_old = copy.copy(jointAngles_L)
            jointAngles_R_old = copy.copy(jointAngles_R)
            i = i + 1 
            timestamp = time.time()

            if (i >= (traj_samples-1)):
                break # break out of while loop


# check if end pose for right arm has been reached
n = 0
while n < 10:
    robot_msg_R = egm_client_R.receive_msg()
    if robot_msg_R.mciConvergenceMet:
        print("Joint positions  for right arm converged.")
        break
    else:
        print("Waiting for convergence.")
    n += 1
    time.sleep(0.1)
else:
    raise TimeoutError(f"Joint positions for the right arm did not converge.")

# check if poses have been reached for the left arm
n = 0
while n < 10:
    robot_msg_L = egm_client_L.receive_msg()
    if robot_msg_L.mciConvergenceMet:
        print("Joint positions for left arm converged.")
        break
    else:
        print("Waiting for convergence.")
    n += 1
    time.sleep(0.1)
else:
    raise TimeoutError(f"Joint positions for the left arm did not converge.")