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
UDP_PORT_RIGHT = 6511
rate = 80

# take the first desired taskSpaceInput for the right arm and simulate force sensor data 
trans_const = copy.copy(p2[0, :])
#rot_const = copy.copy(phi_delta[0, :])
rot_const = np.array([0, 0, 0])
desPose_R_const = np.concatenate((trans_const, rot_const), axis=0) 

egm_client_R = EGMClient(port=UDP_PORT_RIGHT)

yumi_right = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_right.urdf")
yumi_right.set_operationPoint(0.7)
yumi_right.set_kp(0.2)
yumi_right.set_hybridControl(True)

desVelocities_R_const = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
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

while True and arduino.isOpen():

    # check for new force data
    if arduino.in_waiting: # get the number of bytes in the input buffer
        packet = arduino.readline() # type: bytes  
        str_receive = packet.decode('utf-8').rstrip('\n')
        force = float(str_receive)/1000.0 # mN to Newtion

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
        yumi_right.set_desPoseVel(desPose_R_const, desVelocities_R_const)
        yumi_right.set_force(force)
        yumi_right.process()

        print(force)

        ik_jointAngles_R = yumi_right.get_newJointValues() # computed joint values from IK

        #print(f"Current configuration of left arm {conf_yumi_L}")
        #print(f"Current configuration of right arm {conf_yumi_R}")

        # transform to degrees as egm wants it
        des_conf_R = np.degrees(ik_jointAngles_R)

        egm_client_R.send_planned_configuration(logic2abb(des_conf_R))


        # save joint values to compute joint velocities in the next iteration
        jointAngles_R_old = copy.copy(jointAngles_R)

        i = i+1
        timestamp = time.time()
        ##time.sleep(1/rate) 



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
