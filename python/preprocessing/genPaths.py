import numpy as np
from numpy.linalg import norm
from math import sqrt, cos, sin 

# read g-code data from txt-file
with open('gcode.txt') as f:
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
# 3) rotate tcp 2 back in world frame

# split pos 
pos_split = np.split(pos, 2, axis=1)
pos1, pos2 = pos_split[0], pos_split[1]
# number of points
pNum = len(pos[:,0])
# length of wire
wLen = 800
z1 = np.zeros((pNum,1))
z2 = np.ones((pNum,1)) * wLen
# append z axis to position vectors
pos1_ = np.hstack((pos1,z1))
pos2_ = np.hstack((pos2,z2))
pos2_ref = np.hstack((pos2,z2)) # original coordinates
# compute distance between effectors
effLen = np.zeros((pNum,1))
# store rotation around x and y axis
ang = np.zeros((pNum,2))

for i in range(pNum):
    x1, x2 = pos1[i,0], pos2[i,0]
    y1, y2 = pos1[i,1], pos2[i,1]
    dx = x2 - x1
    dy = y2 - y1
    hyp1 = sqrt(abs(dx)**2 + wLen**2) # calculate first hypothenuse
    hyp2 = sqrt(abs(dy)**2 + hyp1**2)
    ang[i, 0] = np.arcsin(dy/hyp2)
    ang[i, 1] = np.arcsin(dx/hyp2)
    effLen[i, 0] = hyp2
    dLen = hyp2 - wLen
    
    # compute rotation matrices
    Rx = np.array([[1, 0, 0], 
              [0, cos(ang[i,0]), sin(ang[i,0])],
              [0, -sin(ang[i,0]), cos(ang[i,0])]])

    Ry = np.array([[cos(ang[i,1]), 0, -sin(ang[i,1])], 
              [0, 1, 0],
              [sin(ang[i,1]), 0, cos(ang[i,1])]])

    # rotation matrix
    R01 = Rx @ Ry

    # build transformation matrix (rotation and translation)
    # r02 = A01 x r12
    r01 = np.array([[x1], [y1], [0]])
    r12 = np.array([[0], [0], [wLen], [1]])
    A01 = np.hstack((R01, r01))
    A01 = np.vstack((A01, np.array([0, 0, 0, 1])))
    r02 = A01 @ r12

    # store values, cut last entry being 1 that was added for the transformation
    pos2_[i,:] = np.transpose(r02[0:3])

# do some tests the check for correctness
# check if distance between two final points matches wires length
r_12 = pos2_ - pos1_
dist_12 = list(map(lambda x: norm(x, 2), r_12))
# check if distance between computed second point and original one matches effective length
r_12_ = pos2_ref - pos2_
dist_22_ = list(map(lambda x: norm(x, 2), r_12_))

for i, (r12,r22_) in enumerate(zip(dist_12, dist_22_)):
    assert np.abs(r12 - wLen) < 0.001, "path coordinate of p2 violates wire's constraint"
    assert (effLen[i, 0] - wLen - r22_) < 0.001, "path coordinate of p2 violates against to be compensated length difference"



