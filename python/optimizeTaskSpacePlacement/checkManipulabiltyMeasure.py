from data.get_data import get_trajectory, transform2yumi_workspace, place_trajectory
import numpy as np
import libs.invKin as invKin
import copy

import matplotlib.cm as cmx
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import matplotlib

# READ IN THE TRAJECTORY
# get the data in the yumi workspace
p1, v1, p2, v2, phi_delta, dphi = get_trajectory()
p1, v1, p2, v2, phi_delta, dphi = transform2yumi_workspace(p1, v1, p2, v2, phi_delta, dphi)

# define staring postition in workspace for left arm - found by try and error in RS
p1_start_des = np.array([0.3, 0.2, 0.2])
p1, p2 = place_trajectory(p1_start_des, p1, p2)

rate = 1.0/80.0

yumi_right = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_right.urdf")
yumi_left = invKin.Yumi("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_left.urdf")

yumi_right.set_additionalManipConstraint(True)
#yumi_right.set_nullspaceWeight(100.0)
yumi_left.set_additionalManipConstraint(True)
#yumi_left.set_nullspaceWeight(100.0)

manipulability_R = np.zeros((len(p1)))
manipulability_L = np.zeros((len(p1)))
manipulabilityMin_R = np.zeros((3, 3, 3))
manipulabilityMin_L = np.zeros((3, 3, 3))

jointAngles_R = np.array([-110.0, 29.85, 35.92, 49.91, 117.0, 123.0, -117.0]) * np.pi/180.0 
jointAngles_L = np.array([90.48, 17.87, -25.09, 48.0, -137.0, 122.0, -74.21]) * np.pi/180.0

jointVelocities_L = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
jointVelocities_R = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

myX_R = []
myY_R = []
myZ_R = []
myMan_R = []

myX_L = []
myY_L = []
myZ_L = []
myMan_L = []

smallestMan = []

oldPos = np.zeros((3))
newPos = np.zeros((3))

for x in range(-20, 21, 5):
    # add 5 cm to every point of trajectory in x direction
    dx = np.array([x*0.01, 0.0, 0.0])
    dX = np.ones((len(p1), 3)) * dx # elementwise multiplication
    
    for y in range(-20, 21, 5):
        dy = np.array([0.0, y*0.01, 0.0])
        dY = np.ones((len(p1), 3)) * dy # elementwise multiplication

        for z in range(-10, 31, 5):
            dz = np.array([0.0, 0.0, z*0.01])
            dZ = np.ones((len(p1), 3)) * dz # elementwise multiplication
            p1_ = copy.copy(p1) + dX + dY + dZ
            p2_ = copy.copy(p2) + dX + dY + dZ

            # synchronize joints angles to new start position
            newPos = [dx[0], dy[1], dz[2]]
            vel2NewStartPoint = (newPos - oldPos) * rate
            desPose_L = np.concatenate((p1_[0,:], phi_delta[0,:]), axis=0) 
            desVelocities_L = np.concatenate((vel2NewStartPoint, np.array([0, 0, 0])), axis=0) # rotation does not change

            desPose_R = np.concatenate((p2_[0,:], phi_delta[0,:]), axis=0) 
            desVelocities_R = np.concatenate((vel2NewStartPoint, np.array([0, 0, 0])), axis=0) 

            yumi_left.set_jointValues(jointAngles_L, jointVelocities_L)
            yumi_left.set_desPoseVel(desPose_L, desVelocities_L)
            yumi_left.process()

            yumi_right.set_jointValues(jointAngles_R, jointVelocities_R)
            yumi_right.set_desPoseVel(desPose_R, desVelocities_R)
            yumi_right.process()

            jointAngles_L = yumi_left.get_newJointValues() # computed joint values from IK
            jointAngles_R = yumi_right.get_newJointValues() # computed joint values from IK


            for index, (pos1, vel1, pos2, vel2, phi, phi_dot) in enumerate(zip(p1_, v1, p2_, v2, phi_delta, dphi)):

                # compute the resulting jointVelocities
                if index > 0:
                    jointVelocities_L = (jointAngles_L - jointAngles_L_old)/rate
                    jointVelocities_R = (jointAngles_R - jointAngles_R_old)/rate

                desPose_L = np.concatenate((pos1, phi), axis=0) 
                desVelocities_L = np.concatenate((vel1, phi_dot), axis=0) 

                desPose_R = np.concatenate((pos2, phi), axis=0) 
                desVelocities_R = np.concatenate((vel2, phi_dot), axis=0) 


                yumi_left.set_jointValues(jointAngles_L, jointVelocities_L)
                yumi_left.set_desPoseVel(desPose_L, desVelocities_L)
                yumi_left.process()
                manipulability_L[index] = yumi_left.get_manipulabilityMeasure()

                yumi_right.set_jointValues(jointAngles_R, jointVelocities_R)
                yumi_right.set_desPoseVel(desPose_R, desVelocities_R)
                yumi_right.process()
                manipulability_R[index] = yumi_right.get_manipulabilityMeasure()

                jointAngles_L = yumi_left.get_newJointValues() # computed joint values from IK
                jointAngles_R = yumi_right.get_newJointValues() # computed joint values from IK

                # save joint values to compute joint velocities in the next iteration
                jointAngles_L_old = copy.copy(jointAngles_L)
                jointAngles_R_old = copy.copy(jointAngles_R)

            #manipulabilityMin_L[x, y, z] = np.min(manipulability_L)
            #manipulabilityMin_R[x, y, z] = np.min(manipulability_R)
            myX_R.append(p2_[0, 0])
            myY_R.append(p2_[0, 1])
            myZ_R.append(p2_[0, 2])
            myMan_R.append(np.min(manipulability_R))

            myX_L.append(p1_[0, 0])
            myY_L.append(p1_[0, 1])
            myZ_L.append(p1_[0, 2])
            myMan_L.append(np.min(manipulability_L))

            if (np.min(manipulability_R) < np.min(manipulability_L)): # if right is smaller
                smallestMan.append(np.min(manipulability_R))
            else:
                smallestMan.append(np.min(manipulability_L))

#print("manipulability measure min right: ", manipulabilityMin_R)
#print("manipulability measure min left: ", manipulabilityMin_L)

plot_path = '/home/joschua/Coding/forceControl/master-project/python/plots/taskSpacePlacement/'
np.save(plot_path +'myX_L_nullgradient', myX_L)
np.save(plot_path +'myY_L_nullgradient', myY_L)
np.save(plot_path +'myZ_L_nullgradient', myZ_L)
np.save(plot_path +'smallestMan_nullgradient', smallestMan)


""" def scatter3d(x,y,z, cs, colorsMap='jet'):
    cm = plt.get_cmap(colorsMap)
    cNorm = matplotlib.colors.Normalize(vmin=min(cs), vmax=max(cs))
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cm)
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(x, y, z, c=scalarMap.to_rgba(cs))
    scalarMap.set_array(cs)
    fig.colorbar(scalarMap)
    plt.show() """


#scatter3d(np.array(myX_R), np.array(myY_R), np.array(myZ_R), np.array(myMan_R))
#scatter3d(np.array(myX_L), np.array(myY_L), np.array(myZ_L), np.array(myMan_L))
#scatter3d(np.array(myX_L), np.array(myY_L), np.array(myZ_L), np.array(myMan_L)+np.array(myMan_R))
# do not add, but search iterate through and choose index where both array entries are higher
