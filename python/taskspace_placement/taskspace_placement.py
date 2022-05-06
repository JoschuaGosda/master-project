import math
import sys
import os

import numpy as np
import copy
import example

# READ IN THE TRAJECTORY
# define staring postition in workspace for left arm
desp_start = np.array([0.35, 0.25, 0.2])

# import the preprocessing data
data = np.load('/home/joschua/Coding/forceControl/master-project/python/taskspace_placement/traj_data.npy')
# for each var x | y | z
p1 = data[:, 0:3]
v1 = data[:, 3:6]
p2 = data[:, 6:9]
v2 = data[:, 9:12]
phi = data[:, 12:15]
dphi = data[:, 15:18]

# coordinates system differ and need to be synchronized - y -> z, x -> x, z -> -y
for m in [p1, v1, p2, v2, phi, dphi]:
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



# TODO create iteration loops for hyperparameter in inverse kinematics
# parameters that can be tuned
weightingFactors = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
activationFactor = 1.0
dt = 0.05

# START CONFIGURATION FOR THE RIGHT ARM
# set the joint angles that map to the desired start position - read from RobotStudio
#jointAngles = np.array([132.0, 40.05, -91.92, 24.75, 285.0, -27.74, -141.0]) * np.pi/180.0
jointAngles = np.array([0.0, 0.0, 0.0, 60.0, 0.0, 0.0, 135.0]) * np.pi/180.0
# initial jointVelocites are zero 
jointVelocities = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# TODO: START CONFIGURATION FOR THE LEFT ARM

phi_robo = np.array([80.0, 100.0, 180.0]) * np.pi/180.0 # values that FK computed, keep angle the same for now, just care about positions
dphi_robo = np.array([0.0, 0.0, 0.0]) * np.pi/180.0
# the current position should match px[0,:] and the desired next position is px[1,:]
desPosition = np.concatenate((p1[1,:], phi_robo), axis=0)
desVelocities = np.concatenate((v1[1,:], dphi_robo), axis=0)

# TODO call the c++ egm function to get the neccessary joint values from pose in taskspace

result = example.gpm(desPosition, desVelocities, jointAngles, jointVelocities, \
    weightingFactors, activationFactor, dt, 1)


newJointAngles= result[0]
lastPose = result[1]

print(newJointAngles)


# TODO compare the errors and closeness to singularities (can use square sum for the errors)