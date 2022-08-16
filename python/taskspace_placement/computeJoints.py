

from matplotlib import pyplot as plt

import numpy as np
import copy
import libs.invKin as invKin

from data.get_data import get_trajectory, transform2yumi_workspace, place_trajectory


# READ IN THE TRAJECTORY
# get the data in the yumi workspace
p1, v1, p2, v2, phi_delta, dphi = get_trajectory()
p1, v1, p2, v2, phi_delta, dphi = transform2yumi_workspace(p1, v1, p2, v2, phi_delta, dphi)

# define staring postition in workspace for left arm - found by try and error in RS
#x: 0.25, y: 0.1, z: 0.05
p1_start_des = np.array([0.35, 0.1, 0.15])
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
compJointAngles_left = np.zeros((len(p1[:,0]),7))
computedPose_left = np.zeros((len(p1[:,0]),6))
error_left = np.zeros((len(p1[:,0]),6))
jointVelocities_left = np.zeros((len(p1[:,0]),7))

yumi_right = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_right.urdf")
yumi_right.set_driftCompGain(1.0)
yumi_right.set_additionalManipConstraint(True)
yumi_left = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_left.urdf")
yumi_left.set_driftCompGain(1.0)
yumi_left.set_additionalManipConstraint(True)

desPose_old = np.concatenate((p1[0,: ], phi_delta[0, :]), axis=0) 

# loop for the left arm
for index, (pos, vel, phi, phi_dot) in enumerate(zip(p1, v1, phi_delta, dphi)): # loop through all the desired position of left arm
    desPose = np.concatenate((pos, phi), axis=0) 
    desVelocities = np.concatenate((vel, phi_dot), axis=0) 
    # call the c++ egm function, return joint values and resulting pose
    yumi_left.set_jointValues(jointAngles, jointVelocities)
    yumi_left.set_desPoseVel(desPose_old, desVelocities)
    yumi_left.process()

    compJointAngles_left[index,:] = yumi_left.get_newJointValues() # computed joint values from IK
    computedPose_left[index, :] = yumi_left.get_pose() # resulting pose with joint values from IK
    if index > 0:
        jointVelocities = (compJointAngles_left[index, :] - compJointAngles_left[index-1, :])/dt # only true in the ideal case where result of ik matches the desired pose
    error_left[index, :] = desPose - computedPose_left[index, :]
    jointVelocities_left[index, :] = jointVelocities
    jointAngles = compJointAngles_left[index, :] 

    # save desired pose to give it to ik in the next iteration since drift compensation must consider if current task space pose is the old desired task space pose
    desPose_old = copy.copy(desPose)



compJointAngles_right = np.zeros((len(p1[:,0]),7))
computedPose_right = np.zeros((len(p1[:,0]),6))
error_right = np.zeros((len(p1[:,0]),6))
jointVelocities_right = np.zeros((len(p1[:,0]),7))

jointAngles = np.array([-110.0, 29.85, 35.92, 49.91, 117.0, 123.0, -117.0]) * np.pi/180.0 
jointVelocities_ = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

desPose_old = np.concatenate((p2[0,: ], phi_delta[0, :]), axis=0) 

# loop for the right arm
for index, (pos, vel, phi, phi_dot) in enumerate(zip(p2, v2, phi_delta, dphi)): # loop through all the desired position of left arm
    desPose = np.concatenate((pos, phi), axis=0) 
    desVelocities = np.concatenate((vel, phi_dot), axis=0) 
    yumi_right.set_jointValues(jointAngles, jointVelocities_)
    yumi_right.set_desPoseVel(desPose_old, desVelocities)
    yumi_right.process()

    compJointAngles_right[index, :] = yumi_right.get_newJointValues() # computed joint values from IK
    computedPose_right[index, :] = yumi_right.get_pose() # resulting pose with joint values from IK
    
    if index > 0:
        jointVelocities_ = (compJointAngles_right[index, :] - compJointAngles_right[index-1, :])/dt 
    error_right[index, :] = desPose - computedPose_right[index, :]
    jointVelocities_right[index, :] = jointVelocities_
    jointAngles = compJointAngles_right[index, :]  

    # save desired pose to give it to ik in the next iteration since drift compensation must consider if current task space pose is the old desired task space pose
    desPose_old = copy.copy(desPose)


plot_path = '/home/joschua/Coding/forceControl/master-project/python/plots/ik_tuning/'
np.save(plot_path +'error_left', error_left)
np.save(plot_path +'error_right', error_right)
np.save(plot_path +'computedPose_left', computedPose_left)
np.save(plot_path +'computedPose_right', computedPose_right)
np.save(plot_path +'compJointAngles_right', compJointAngles_right)
np.save(plot_path +'compJointAngles_left', compJointAngles_left)
np.save(plot_path + 'jointVelocities_right', jointVelocities_right)
np.save(plot_path + 'jointVelocities_left', jointVelocities_left)
np.save(plot_path +'p1', p1)
np.save(plot_path +'p2', p2)






#np.save('data/compJointAngles_left', compJointAngles_left)
#np.save('data/compJointAngles_right', compJointAngles_right)

