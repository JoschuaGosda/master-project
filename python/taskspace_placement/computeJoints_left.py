from enum import Enum
from matplotlib import pyplot as plt

import numpy as np
import copy
import invKin

class Yumi(Enum):
    RIGHT = False
    LEFT = True

# READ IN THE TRAJECTORY
# define staring postition in workspace for left arm - found by try and error in RS
desp_start = np.array([0.3, 0.2, 0.2])

# import the preprocessing data
data = np.load('/home/joschua/Coding/forceControl/master-project/python/taskspace_placement/traj_data.npy')
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


phi_const = np.array([90.0, 180.0, 90.0]) * np.pi/180.0 # values that FK computed, keep angle the same for now, just care about positions
dphi_const = np.array([0.0, 0.0, 0.0]) * np.pi/180.0

# build the final angles for the orientation of end effector - don't use it for now
phi_base = np.zeros((len(p1[:,0]),3)) + phi_const
phi_total = phi_base + phi_delta

# create arrays to store values of loop
desJointAngles = np.zeros((len(p1[:,0]),7))
computedPose = np.zeros((len(p1[:,0]),6))
error = np.zeros((len(p1[:,0]),6))

for index, (pos, vel, phi, phi_dot) in enumerate(zip(p1, v1, phi_total, dphi)): # loop through all the desired position of left arm
    desPose = np.concatenate((pos, phi), axis=0) # note that phi_const is used -> same orientation throughout the movement
    desVelocities = np.concatenate((vel, phi_dot), axis=0) # same here
    # call the c++ egm function, return joint values and resulting pose
    result = invKin.gpm(desPose, desVelocities, jointAngles, jointVelocities, Yumi.LEFT.value)
    desJointAngles[index,:] = result[0] # computed joint values from IK
    computedPose[index, :] = result[1] # resulting pose with joint values from IK
    if index > 0:
        jointVelocities = (desJointAngles[index, :] - desJointAngles[index-1, :])/dt # only true in the ideal case where result of ik matches the desired pose
    #print('IK joints:',  result[0])
    #print('IK resulting pose',  result[1])
    print('\n error', desPose - result[1])
    error[index, :] = desPose - result[1]
    jointAngles = result[0]

# see development of joint values
fig = plt.figure()
plt.plot(desJointAngles[:,0], label='joint1')
plt.plot(desJointAngles[:,1], label='joint2')
plt.plot(desJointAngles[:,2], label='joint3')
plt.plot(desJointAngles[:,3], label='joint4')
plt.plot(desJointAngles[:,4], label='joint5')
plt.plot(desJointAngles[:,5], label='joint6')
plt.plot(desJointAngles[:,6], label='joint7')
plt.title('joint values over samples, left arm')
plt.legend()
plt.show()

# error
fig = plt.figure()
plt.plot(error[:,0], label='x')
plt.plot(error[:,1], label='y')
plt.plot(error[:,2], label='z')
plt.plot(error[:,3], label='rx')
plt.plot(error[:,4], label='ry')
plt.plot(error[:,5], label='rz')
plt.legend()
plt.title('errors')
plt.show()

# show trajectory in workspace of yumi
fig = plt.figure()
plt.plot(p1[:,0], p1[:,2], label='desired profile') # plot z over x
plt.scatter(computedPose[:,0], computedPose[:,2], label='resulting pose from IK')
fig.get_axes()[0].set_xlabel('x axis of yumi')
fig.get_axes()[0].set_ylabel('z axis of yumi')
plt.legend()
plt.title('view on the x-z plane from the right arm side of yumi')
plt.show()

fig = plt.figure()
plt.plot(computedPose[:,3], label='rx')
plt.plot(computedPose[:,4], label='ry')
plt.plot(computedPose[:,5], label='rz')
plt.legend()
plt.title('euler angles over trajectories')
plt.show()

np.save('desJointAngles_left', desJointAngles)

