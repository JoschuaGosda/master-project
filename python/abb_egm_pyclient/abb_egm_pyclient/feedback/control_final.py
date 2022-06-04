# TODO: check if all devices and neccessary files are online/in the right place
# show if thats the case and ask user if he wants to start control 

#!/usr/bin/env python
import keyboard
from typing import Sequence
import time
import copy
import numpy as np
from abb_egm_pyclient import EGMClient
import libs.invKin as invKin
from data.get_data import get_trajectory, transform2yumi_workspace, place_trajectory, logic2abb
from matplotlib import pyplot as plt
import serial.tools.list_ports
from tqdm import tqdm
from stateMachine import FoamCuttingMachine


foamCutting = FoamCuttingMachine()

def get_realJointAngles(egm_client):
    robot_msg = egm_client.receive_msg()
    conf: Sequence[float] = robot_msg.feedBack.joints.joints
    joint7 = robot_msg.feedBack.externalJoints.joints[0]
    conf.insert(2, joint7)
    jointAngles = np.radians(np.array(conf))
    return jointAngles

# READ IN THE TRAJECTORY
# get the data in the yumi workspace
p1, v1, p2, v2, phi_delta, dphi = get_trajectory()
p1, v1, p2, v2, phi_delta, dphi = transform2yumi_workspace(p1, v1, p2, v2, phi_delta, dphi)

# define staring postition in workspace for left arm - found by try and error in RS
p1_start_des = np.array([0.32, 0.3, 0.1]) # synced fine pose was np.array([0.3, 0.2, 0.2])
p1, p2 = place_trajectory(p1_start_des, p1, p2)

# setup for UDP connection
UDP_PORT_LEFT = 6510
UDP_PORT_RIGHT = 6511
rate = 80

# take the first desired taskSpaceInput for the right arm and simulate force sensor data 
#translation_R_start = copy.copy(p2[0, :])
syncTranslation_R = np.array([0.3, -0.2,  0.2]) # starting pose synched with robot
#rotation_R_start = copy.copy(phi_delta[0, :])
syncRotation_R = np.array([0, 0, 0])
syncPose_R = np.concatenate((syncTranslation_R, syncRotation_R), axis=0) 
desPose_R_start = np.concatenate((p2[0, :], phi_delta[0, :]), axis=0) 

#translation_L_start = copy.copy(p1[0, :])
syncTranslation_L = np.array([0.3, 0.2, 0.2]) # starting pose synched with robot
syncRotation_L = np.array([0, 0, 0])
syncPose_L = np.concatenate((syncTranslation_L, syncRotation_L), axis=0)
desPose_L_start = np.concatenate((p1[0, :], phi_delta[0, :]), axis=0)

egm_client_R = EGMClient(port=UDP_PORT_RIGHT)
egm_client_L = EGMClient(port=UDP_PORT_LEFT)

yumi_right = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_right.urdf")
yumi_right.set_operationPoint(1.0)
yumi_right.set_kp(0.05)
yumi_right.set_hybridControl(False)
yumi_right.set_transitionTime(3.0)

yumi_left = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_left.urdf")

desVelocities_R_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
jointVelocities_R = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

desVelocities_L_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
jointVelocities_L = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

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
traj_samples = len(p1[:, 0]) 
moveToStartVel = 0.005

# delta is for both arms the same
poseDelta = desPose_R_start - syncPose_R

numIterations = round(np.fabs(poseDelta).max()/moveToStartVel * rate)
toStartPose_R = np.zeros((numIterations, 6))
toStartPose_L = np.zeros((numIterations, 6))

for i in range(numIterations):
    toStartPose_L[i, :] = syncPose_L + poseDelta/numIterations * i
    toStartPose_R[i, :] = syncPose_R + poseDelta/numIterations * i

toStartVel_R = (toStartPose_R[1, :] - toStartPose_R[0, :]) * rate
toStartVel_L = (toStartPose_L[1, :] - toStartPose_L[0, :]) * rate

# arrays to store the results for later plotting
log_realPose_R = np.zeros((traj_samples, 6))
log_compPose_R = np.zeros((traj_samples, 6))
log_realJoints_R = np.zeros((traj_samples, 7))
log_compJoints_R = np.zeros((traj_samples, 7))

log_realPose_L = np.zeros((traj_samples, 6))
log_compPose_L = np.zeros((traj_samples, 6))
log_realJoints_L = np.zeros((traj_samples, 7))
log_compJoints_L = np.zeros((traj_samples, 7))

log_force = np.zeros((traj_samples, 1))

print("\n Force control only to tension the wire...")

foamCutting.start_syncronizing()

i = 0
while True and arduino.isOpen():
    # check for new force data
    if arduino.in_waiting: # get the number of bytes in the input buffer
        packet = arduino.readline() # type: bytes  
        str_receive = packet.decode('utf-8').rstrip('\n')
        force = float(str_receive)/1000.0 # mN to Newton

    if foamCutting.is_moveToStartPose:
        if (time.time() - timestamp) >= (1.0/rate):
            timestamp = time.time()

            # get the current joints angles for the right arm
            jointAngles_R = get_realJointAngles(egm_client_R)
            jointAngles_L = get_realJointAngles(egm_client_L)

            # compute the resulting jointVelocities
            if i > 0:
                jointVelocities_R = (jointAngles_R - jointAngles_R_old)/rate
                jointVelocities_L = (jointAngles_L - jointAngles_L_old)/rate

            yumi_right.set_jointValues(jointAngles_R, jointVelocities_R)
            yumi_right.set_desPoseVel(toStartPose_R[i, :], toStartVel_R)
            yumi_right.process()
            pose_R = yumi_right.get_pose()
            poseError_R = pose_R - syncPose_R

            yumi_left.set_jointValues(jointAngles_L, jointVelocities_L)
            yumi_left.set_desPoseVel(toStartPose_L[i, :], toStartVel_L)
            yumi_left.process()
            pose_L = yumi_left.get_pose()
            poseError_L = pose_L - syncPose_L

            ik_jointAngles_R = yumi_right.get_newJointValues() # computed joint values from IK
            ik_jointAngles_L = yumi_left.get_newJointValues()

            # transform to degrees as egm wants it
            des_conf_R = np.degrees(ik_jointAngles_R)
            des_conf_L = np.degrees(ik_jointAngles_L)

            egm_client_R.send_planned_configuration(logic2abb(des_conf_R))
            egm_client_L.send_planned_configuration(logic2abb(des_conf_L))

            # save joint values to compute joint velocities in the next iteration
            jointAngles_R_old = copy.copy(jointAngles_R)
            jointAngles_L_old = copy.copy(jointAngles_L)

            i = i+1
            if i == numIterations:
                print('end effectors are at starting poses, ready to mount wire')
                i = 0 # reset counter
                foamCutting.start_mounting()
                
    elif (foamCutting.is_mountWire):
        if keyboard.is_pressed('enter'):
            yumi_right.set_hybridControl(True)
            foamCutting.start_tensioning()


    # force control only until wire is tensioned
    elif foamCutting.is_tensionWire:
        if (time.time() - timestamp) >= (1.0/rate):
            timestamp = time.time()

            # get the current joints angles for the right arm
            jointAngles_R = get_realJointAngles(egm_client_R)
            jointAngles_L = get_realJointAngles(egm_client_L)

            # compute the resulting jointVelocities
            if i > 0:
                jointVelocities_R = (jointAngles_R - jointAngles_R_old)/rate
                jointVelocities_L = (jointAngles_L - jointAngles_L_old)/rate

            yumi_right.set_jointValues(jointAngles_R, jointVelocities_R)
            yumi_right.set_desPoseVel(desPose_R_start, desVelocities_R_start)
            yumi_right.set_force(force)
            yumi_right.process()

            yumi_left.set_jointValues(jointAngles_L, jointVelocities_L)
            yumi_left.set_desPoseVel(desPose_L_start, desVelocities_L_start)
            yumi_left.process()

            print("force in [N] is: %5.2f" %  force)

            ik_jointAngles_R = yumi_right.get_newJointValues() # computed joint values from IK
            ik_jointAngles_L = yumi_left.get_newJointValues()

            # transform to degrees as egm wants it
            des_conf_R = np.degrees(ik_jointAngles_R)
            des_conf_L = np.degrees(ik_jointAngles_L)

            egm_client_R.send_planned_configuration(logic2abb(des_conf_R))
            egm_client_L.send_planned_configuration(logic2abb(des_conf_L))

            # save joint values to compute joint velocities in the next iteration
            jointAngles_R_old = copy.copy(jointAngles_R)
            jointAngles_L_old = copy.copy(jointAngles_L)

            i = i+1
            if (force > 0.65) and keyboard.is_pressed('enter'):
                i = 0 # reset counter
                print("Changing to hybrid control now...")
                foamCutting.start_cutting()
                pbar = tqdm(total=traj_samples)
    
    # hybrid control 
    elif foamCutting.is_cutting:
        if (time.time() - timestamp) >= (1.0/rate):
            timestamp = time.time()
            # copy array entries to local variables
            pos1 = p1[i, :]
            vel1 = v1[i, :]
            pos2 = p2[i, :]
            vel2 = v2[i, :]
            phi = phi_delta[i, :]
            phi_dot = dphi[i, :]

            # get the current joints angles for the left arm
            jointAngles_L = get_realJointAngles(egm_client_L)
            log_realJoints_L[i, :] = jointAngles_L

            # get the current joints angles for the right arm
            jointAngles_R = get_realJointAngles(egm_client_R)
            log_realJoints_R[i, :] = jointAngles_R

            # compute the resulting jointVelocities
            if i > 0:
                jointVelocities_L = (jointAngles_L - jointAngles_L_old)/rate
                jointVelocities_R = (jointAngles_R - jointAngles_R_old)/rate

            desPose_L = np.concatenate((pos1, phi), axis=0) 
            desVelocities_L = np.concatenate((vel1, phi_dot), axis=0) 

            desPose_R = np.concatenate((pos2, phi), axis=0) 
            desVelocities_R = np.concatenate((vel2, phi_dot), axis=0) 

            yumi_left.set_jointValues(jointAngles_L, jointVelocities_L)
            log_realPose_L[i, :] = yumi_left.get_pose()
            yumi_left.set_desPoseVel(desPose_L, desVelocities_L)
            yumi_left.process()
            log_compPose_L[i, :] = yumi_left.get_pose()

            yumi_right.set_jointValues(jointAngles_R, jointVelocities_R)
            log_realPose_R[i, :] = yumi_right.get_pose()
            yumi_right.set_desPoseVel(desPose_R, desVelocities_R)
            yumi_right.set_force(force)
            yumi_right.process()
            log_compPose_R[i, :] = yumi_right.get_pose()
            log_force[i, :] = force

            ik_jointAngles_L = yumi_left.get_newJointValues() # computed joint values from IK
            log_compJoints_L[i, :] = ik_jointAngles_L
            ik_jointAngles_R = yumi_right.get_newJointValues() # computed joint values from IK
            log_compJoints_R[i, :] = ik_jointAngles_R

            # transform to degrees as egm wants it
            des_conf_L = np.degrees(ik_jointAngles_L)
            des_conf_R = np.degrees(ik_jointAngles_R)

            egm_client_L.send_planned_configuration(logic2abb(des_conf_L))
            egm_client_R.send_planned_configuration(logic2abb(des_conf_R))

            # save joint values to compute joint velocities in the next iteration
            jointAngles_L_old = copy.copy(jointAngles_L)
            jointAngles_R_old = copy.copy(jointAngles_R)
            i = i + 1 
            pbar.update(1)
            
            if (i >= (traj_samples-1)):
                pbar.close()
                foamCutting.end_cutting()
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


experimentLogs = np.hstack((p2, phi_delta, log_compPose_R, log_realPose_R, log_compJoints_R, log_realJoints_R, log_force, p1, log_compJoints_R, log_realPose_L, log_compJoints_L, log_realJoints_L))
#np.save('./data/experimentLogs300-0,05-1-x', experimentLogs)