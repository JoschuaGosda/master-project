import numpy as np
import example

# TODO import the preprocessing data
#p1, v1, p2, v2, phi, dphi = np.load('traj_data.npy')
# data = np.load('traj_data.npy')
# TODO place the trajectories within the workspace of the robot

# TODO create iteration loops for hyperparameter in inverse kinematics
# parameters that can be tuned
weightingFactors = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
activationFactor = 1.0
dt = 0.05

jointAngles = np.array([0.0, 0.0, 0.0, np.deg2rad(-60), 0.0, 0.0, np.deg2rad(45)])
#jointAngles = jointAngles * np.deg2rad

jointVelocities = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

desPosition =  np.array([0.66, -0.193, 0.629, np.deg2rad(97.9), np.deg2rad(-179.9), np.deg2rad(90.78)])
desVelocities = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

# TODO call the c++ egm function to get the neccessary joint values from pose in taskspace
result = example.gpm(desPosition, desVelocities, jointAngles, jointVelocities, \
    weightingFactors, activationFactor, dt, 1)

newJointAngles= result[0]
lastPose = result[1]

print(newJointAngles)


# TODO compare the errors and closeness to singularities (can use square sum for the errors)