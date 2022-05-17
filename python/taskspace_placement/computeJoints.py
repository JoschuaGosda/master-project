

from matplotlib import pyplot as plt

import numpy as np
import copy
from libs.invKin import gpm

from data.get_data import get_trajectory, transform2yumi_workspace, place_trajectory, Yumi


# READ IN THE TRAJECTORY
# get the data in the yumi workspace
p1, v1, p2, v2, phi_delta, dphi = get_trajectory()
p1, v1, p2, v2, phi_delta, dphi = transform2yumi_workspace(p1, v1, p2, v2, phi_delta, dphi)

# define staring postition in workspace for left arm - found by try and error in RS
p1_start_des = np.array([0.3, 0.2, 0.2])
p1, p2 = place_trajectory(p1_start_des, p1, p2)


# START CONFIGURATION FOR THE LEFT ARM
# set the joint angles that map to the desired start position - read from RobotStudio
jointAngles = np.array([90.48, 17.87, -25.09, 48.0, -137.0, 122.0, -74.21]) * np.pi/180.0 #show good manipulability index in RS
#jointAngles = np.array([90.48, 17.87, -25.09, 48.0, -137.0, 122.0, -74.21]) * np.pi/180.0 #show good manipulability index in RS
#jointAngles = np.array([90.48, 17.87, -25.09, 48.0, -137.0, 122.0, 15.79]) * np.pi/180.0
# initial jointVelocites are zero 
jointVelocities = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
dt = 0.0125

# create arrays to store values of loop
desJointAngles_left = np.zeros((len(p1[:,0]),7))
computedPose_left = np.zeros((len(p1[:,0]),6))
error_left = np.zeros((len(p1[:,0]),6))

# loop for the left arm
for index, (pos, vel, phi, phi_dot) in enumerate(zip(p1, v1, phi_delta, dphi)): # loop through all the desired position of left arm
    desPose = np.concatenate((pos, phi), axis=0) 
    desVelocities = np.concatenate((vel, phi_dot), axis=0) 
    # call the c++ egm function, return joint values and resulting pose
    result = gpm(desPose, desVelocities, jointAngles, jointVelocities, Yumi.LEFT.value)
    desJointAngles_left[index,:] = result[0] # computed joint values from IK
    computedPose_left[index, :] = result[1] # resulting pose with joint values from IK
    if index > 0:
        jointVelocities = (desJointAngles_left[index, :] - desJointAngles_left[index-1, :])/dt # only true in the ideal case where result of ik matches the desired pose
    #print('IK joints:',  result[0])
    #print('IK resulting pose',  result[1])
    print('\n error', desPose - result[1])
    error_left[index, :] = desPose - result[1]
    jointAngles = result[0]


desJointAngles_right = np.zeros((len(p1[:,0]),7))
computedPose_right = np.zeros((len(p1[:,0]),6))
error_right = np.zeros((len(p1[:,0]),6))

jointAngles = np.array([-110.0, 29.85, 35.92, 49.91, 117.0, 123.0, -117.0]) * np.pi/180.0 

# loop for the right arm
for index, (pos, vel, phi, phi_dot) in enumerate(zip(p2, v2, phi_delta, dphi)): # loop through all the desired position of left arm
    desPose = np.concatenate((pos, phi), axis=0) 
    desVelocities = np.concatenate((vel, phi_dot), axis=0) 
    result = gpm(desPose, desVelocities, jointAngles, jointVelocities, Yumi.RIGHT.value)
    desJointAngles_right[index,:] = result[0] 
    computedPose_right[index, :] = result[1] 
    if index > 0:
        jointVelocities = (desJointAngles_left[index, :] - desJointAngles_left[index-1, :])/dt 
    print('\n error', desPose - result[1])
    error_right[index, :] = desPose - result[1]
    jointAngles = result[0]

# see development of joint values
fig = plt.figure()
plt.plot(desJointAngles_left[:,0], label='joint1')
plt.plot(desJointAngles_left[:,1], label='joint2')
plt.plot(desJointAngles_left[:,2], label='joint3')
plt.plot(desJointAngles_left[:,3], label='joint4')
plt.plot(desJointAngles_left[:,4], label='joint5')
plt.plot(desJointAngles_left[:,5], label='joint6')
plt.plot(desJointAngles_left[:,6], label='joint7')
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
plt.title('view on the x-z plane from the right arm side of yumi')
plt.show()

fig = plt.figure()
plt.plot(computedPose_left[:,3], label='rx')
plt.plot(computedPose_left[:,4], label='ry')
plt.plot(computedPose_left[:,5], label='rz')
plt.legend()
plt.title('euler angles over trajectories')
plt.show()

fig = plt.figure()
plt.plot(desJointAngles_right[:,0], label='joint1')
plt.plot(desJointAngles_right[:,1], label='joint2')
plt.plot(desJointAngles_right[:,2], label='joint3')
plt.plot(desJointAngles_right[:,3], label='joint4')
plt.plot(desJointAngles_right[:,4], label='joint5')
plt.plot(desJointAngles_right[:,5], label='joint6')
plt.plot(desJointAngles_right[:,6], label='joint7')
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

np.save('data/desJointAngles_left', desJointAngles_left)
np.save('data/desJointAngles_right', desJointAngles_right)

