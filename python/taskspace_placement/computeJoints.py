

from matplotlib import pyplot as plt

import numpy as np
import copy
import libs.invKin as invKin

from data.get_data import get_trajectory, transform2yumi_workspace, place_trajectory, Yumi


# READ IN THE TRAJECTORY
# get the data in the yumi workspace
""" p1, v1, p2, v2, phi_delta, dphi = get_trajectory()
p1, v1, p2, v2, phi_delta, dphi = transform2yumi_workspace(p1, v1, p2, v2, phi_delta, dphi)

# define staring postition in workspace for left arm - found by try and error in RS
p1_start_des = np.array([0.3, 0.2, 0.2])
p1, p2 = place_trajectory(p1_start_des, p1, p2) """

desp_start = np.array([0.3, 0.2, 0.2])

# import the preprocessing data
data = np.load('./data/traj_data.npy')
# for each var x | y | z
p1 = data[:, 0:3]
v1 = data[:, 3:6]
p2 = data[:, 6:9]
v2 = data[:, 9:12]
phi_delta = data[:, 12:15]
dphi = data[:, 15:18]

# coordinates system differ and need to be synchronized - y -> z, x -> x, z -> -y
for m in [p1, v1, p2, v2, phi_delta, dphi]:
    copy_col = copy.copy(m[:, 2]) # copy z
    m[:, 2] = m[:, 1] # shift y to z
    m[:, 1] = -copy_col # copy z to y

# place the trajectories within the workspace of the robot
# read the coordinates of p1 (that refers to the left arm) and modify it to match to desired 
# starting postion
p_start = p1[0, :]
offset = desp_start - p_start

# apply offset to all position coordinates
for i in range(len(p1[:,0])):
    p1[i,:] = p1[i,:] + offset
    p2[i,:] = p2[i,:] + offset



# START CONFIGURATION FOR THE LEFT ARM
# set the joint angles that map to the desired start position - read from RobotStudio
jointAngles = np.array([90.48, 17.87, -25.09, 48.0, -137.0, 122.0, -74.21]) * np.pi/180.0 #show good manipulability index in RS
#jointAngles = np.array([90.48, 17.87, -25.09, 48.0, -137.0, 122.0, -74.21]) * np.pi/180.0 #show good manipulability index in RS
#jointAngles = np.array([90.48, 17.87, -25.09, 48.0, -137.0, 122.0, 15.79]) * np.pi/180.0
# initial jointVelocites are zero 
jointVelocities = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
dt = 0.0125

# create arrays to store values of loop
compJointAngles_left = np.zeros((len(p1[:,0]),7))
computedPose_left = np.zeros((len(p1[:,0]),6))
error_left = np.zeros((len(p1[:,0]),6))

yumi_right = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_right.urdf")
yumi_left = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_left.urdf")


#yumi_left.printPose()

# loop for the left arm
for index, (pos, vel, phi, phi_dot) in enumerate(zip(p1, v1, phi_delta, dphi)): # loop through all the desired position of left arm
    desPose = np.concatenate((pos, phi), axis=0) 
    desVelocities = np.concatenate((vel, phi_dot), axis=0) 
    # call the c++ egm function, return joint values and resulting pose
    yumi_left.set_jointValues(jointAngles, jointVelocities)
    yumi_left.set_desPoseVel(desPose, desVelocities)
    yumi_left.process()

    compJointAngles_left[index,:] = yumi_left.get_newJointValues() # computed joint values from IK
    computedPose_left[index, :] = yumi_left.get_newPose() # resulting pose with joint values from IK
    if index > 0:
        jointVelocities = (compJointAngles_left[index, :] - compJointAngles_left[index-1, :])/dt # only true in the ideal case where result of ik matches the desired pose
    error_left[index, :] = desPose - computedPose_left[index, :]
    jointAngles = compJointAngles_left[index, :] 




compJointAngles_right = np.zeros((len(p1[:,0]),7))
computedPose_right = np.zeros((len(p1[:,0]),6))
error_right = np.zeros((len(p1[:,0]),6))

jointAngles = np.array([-110.0, 29.85, 35.92, 49.91, 117.0, 123.0, -117.0]) * np.pi/180.0 
jointVelocities = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# loop for the right arm
for index, (pos, vel, phi, phi_dot) in enumerate(zip(p2, v2, phi_delta, dphi)): # loop through all the desired position of left arm
    desPose = np.concatenate((pos, phi), axis=0) 
    desVelocities = np.concatenate((vel, phi_dot), axis=0) 
    yumi_right.set_jointValues(jointAngles, jointVelocities)
    yumi_right.set_desPoseVel(desPose, desVelocities)
    yumi_right.process()

    compJointAngles_right[index, :] = yumi_right.get_newJointValues() # computed joint values from IK
    computedPose_right[index, :] = yumi_right.get_newPose() # resulting pose with joint values from IK
    
    if index > 0:
        jointVelocities = (compJointAngles_left[index, :] - compJointAngles_left[index-1, :])/dt 
    error_right[index, :] = desPose - computedPose_right[index, :]
    jointAngles = compJointAngles_right[index, :]  

# see development of joint values
fig = plt.figure()
plt.plot(compJointAngles_left[:,0], label='joint1')
plt.plot(compJointAngles_left[:,1], label='joint2')
plt.plot(compJointAngles_left[:,2], label='joint3')
plt.plot(compJointAngles_left[:,3], label='joint4')
plt.plot(compJointAngles_left[:,4], label='joint5')
plt.plot(compJointAngles_left[:,5], label='joint6')
plt.plot(compJointAngles_left[:,6], label='joint7')
plt.title('joint values over samples, left arm')
plt.legend()
plt.show()

# error
fig = plt.figure()
plt.plot(error_left[:,0], label='x')
plt.plot(error_left[:,1], label='y')
plt.plot(error_left[:,2], label='z')
plt.plot(error_left[:,3], label='rx')
plt.plot(error_left[:,4], label='ry')
plt.plot(error_left[:,5], label='rz')
plt.legend()
plt.title('errors left')
plt.show()

# show trajectory in workspace of yumi
fig = plt.figure()
plt.plot(p1[:,0], p1[:,2], label='desired profile') # plot z over x
plt.scatter(computedPose_left[:,0], computedPose_left[:,2], label='resulting pose from IK')
fig.get_axes()[0].set_xlabel('x axis of yumi')
fig.get_axes()[0].set_ylabel('z axis of yumi')
plt.legend()
plt.title('poses left')
plt.show()

""" fig = plt.figure()
plt.plot(computedPose_left[:,3], label='rx')
plt.plot(computedPose_left[:,4], label='ry')
plt.plot(computedPose_left[:,5], label='rz')
plt.legend()
plt.title('euler angles over trajectories')
plt.show() """

fig = plt.figure()
plt.plot(compJointAngles_right[:,0], label='joint1')
plt.plot(compJointAngles_right[:,1], label='joint2')
plt.plot(compJointAngles_right[:,2], label='joint3')
plt.plot(compJointAngles_right[:,3], label='joint4')
plt.plot(compJointAngles_right[:,4], label='joint5')
plt.plot(compJointAngles_right[:,5], label='joint6')
plt.plot(compJointAngles_right[:,6], label='joint7')
plt.title('joint values over samples, right arm')
plt.legend()
plt.show()

fig = plt.figure()
plt.plot(error_right[:,0], label='x')
plt.plot(error_right[:,1], label='y')
plt.plot(error_right[:,2], label='z')
plt.plot(error_right[:,3], label='rx')
plt.plot(error_right[:,4], label='ry')
plt.plot(error_right[:,5], label='rz')
plt.legend()
plt.title('errors right')
plt.show()

# show trajectory in workspace of yumi
fig = plt.figure()
plt.plot(p2[:,0], p2[:,2], label='desired profile') # plot z over x
plt.scatter(computedPose_right[:,0], computedPose_right[:,2], label='resulting pose from IK')
fig.get_axes()[0].set_xlabel('x axis of yumi')
fig.get_axes()[0].set_ylabel('z axis of yumi')
plt.legend()
plt.title('poses right')
plt.show()

np.save('data/compJointAngles_left', compJointAngles_left)
np.save('data/compJointAngles_right', compJointAngles_right)

