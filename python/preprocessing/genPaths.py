import numpy as np
from numpy.linalg import norm
from math import sqrt, cos, sin 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# read g-code data from txt-file
with open('preprocessing/gcode.txt') as f:
    lines = f.readlines()
    # create array to store path values of x1, y1, x2, y2
    pos = np.zeros((len(lines),4))

    for col, line in enumerate(lines):
        line = line.strip() # remove leading/trailing white space
        line_split = line.split(' ')

        # iterate through elements in line and assign it to pos variable
        for row, str in enumerate(line_split):
            if row > 0:
                pos[col, row-1] = np.asarray(str[1:len(str)]) # cut the first element of string which is not needed

# goal must be to have both tcp positions in the world frame, due to different approach with fixed wire length
# second tcp must be corrected. Can be calculated from the first one based on orientation and length of wire. 
# 1) extract wires orientation from 4 axis approach - two axis with x-y coordinates and fixed distance (in z-direction)
# 2) use the orienation angles to do a rotation transformation and walk fixed distance in wires axis

# extract original x & y of both tcp's
pos_split = np.split(pos, 2, axis=1)
pos1, pos2 = pos_split[0], pos_split[1] 
# number of points in paths
pNum = len(pos[:,0])
# length of wire, defined in 4-axis setup 
wLen = 800
z1 = np.zeros((pNum,1))
z2 = np.ones((pNum,1)) * wLen
# append z axis to position vectors
pos1_ = np.hstack((pos1,z1))
pos2_ = np.hstack((pos2,z2))
pos2_ref = np.hstack((pos2,z2)) # original coordinates
# define array to compute distance between effectors
effLen = np.zeros((pNum,1))
# define array to store rotations around x and y axis; index 0: rotation around x-axis, index 1: rotation around y axis
ang = np.zeros((pNum,2))

for i in range(pNum):
    x1, x2 = pos1[i,0], pos2[i,0]
    y1, y2 = pos1[i,1], pos2[i,1]
    dx = x2 - x1
    dy = y2 - y1
    # compute angles based on geometric evaluation of setup, look at it as cuboid with wire from lower, left, back corner
    # to  upper, right, front corner. Calculate length of wire and angles using basic geometrics. 
    hyp1 = sqrt(dx**2 + wLen**2) # calculate first hypothenuse
    hyp2 = sqrt(dy**2 + hyp1**2) 
    ang[i, 1] = np.arctan(dx/wLen) # rotation around y-axis from oringal frame
    ang[i, 0] = np.arctan(dy/hyp1) # rotation around x-axis in already rotated frame
    effLen[i, 0] = hyp2 # effective length
    dLen = hyp2 - wLen 
    
    # compute rotation matrices
    # rotation in negative direction
    Rx = np.array([[1, 0, 0], 
              [0, cos(ang[i,0]), sin(ang[i,0])],
              [0, -sin(ang[i,0]), cos(ang[i,0])]])

    # rotation in postive direction
    Ry = np.array([[cos(ang[i,1]), 0, sin(ang[i,1])], 
              [0, 1, 0],
              [-sin(ang[i,1]), 0, cos(ang[i,1])]])

    # rotation matrix
    # self defined convention: rotate around y axis, then around x axis the get from  initial frame to wire frame
    # rotation from initial to first frame (z-axis aligned with wire)
    #R01 = Rx @ Ry 
    R01 = Ry @ Rx
    # rotation from first frame to initial frame
    R10 = np.transpose(R01)

    # build transformation matrix (rotation and translation) - r02 = T01 x r12
    # vector 1 in inital frame
    r0_01 = np.array([[x1], [y1], [0]])
    # vector in wires direction in first frame
    #r12 = np.array([[0], [0], [wLen], [1]])
    #T01 = np.hstack((R01, r01))
    #T01 = np.vstack((T01, np.array([0, 0, 0, 1])))

    r1_12 = np.array([[0], [0], [wLen]])
    r0_12 = R01 @ r1_12
    r0_02 = r0_12 + r0_01



    # store values, cut last entry being 1 that was added for the transformation
    pos2_[i,:] = np.transpose(r0_02)

# do some tests that check for correctness
# check if distance between two final points matches wires length
r_12 = pos2_ - pos1_
dist_12 = list(map(lambda x: norm(x, 2), r_12))
# check if distance between computed second point and original one + wire's length matches effective length
r_12_ = pos2_ref - pos2_
dist_22_ = list(map(lambda x: norm(x, 2), r_12_))

for i, (r12,r22_) in enumerate(zip(dist_12, dist_22_)):
    assert np.abs(r12 - wLen) < 0.001, "path coordinate of p2 violates wire's constraint"
    assert (effLen[i, 0] - wLen - r22_) < 0.001, "path coordinate of p2 violates against to be compensated length difference"

# plot the data for visualisation
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(pos1_[:,0],pos1_[:,1],pos1_[:,2])
ax.plot(pos2_[:,0],pos2_[:,1],pos2_[:,2])
ax.plot(pos2_ref[:,0],pos2_ref[:,1],pos2_ref[:,2])
ax.set_xlim(0,170)
ax.set_ylim(0,100)
ax.set_zlim(795,805)

#plt.plot(pos1_[:,0],pos1_[:,2], marker='o')
#plt.plot(pos2_[:,0],pos2_[:,2], marker='o')
#plt.plot(pos2_ref[:,0],pos2_ref[:,2], marker='o')
plt.show()

