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
pos1, pos2 = pos_split[0]*0.001, pos_split[1]*0.001 # transform to SI-units - [m]

# TODO: compute trajectory based on path of g-code
# set a cutting speed, sample rate and compute necessary
# position at every time step
c_speed = 3000 * 0.001 * 1/60 # define cutting speed as 300 mm/min [m/s]
st = 1.0/80.0 # highest possible sample time - 250 Hz


# initialize list with first element
p1x = []
p1y = []
p2x = []
p2y = []

for i in range(1, len(pos1[:,0])):
    if(abs(pos1[i,0] - pos1[i-1,0]) < 1e-6 and abs(pos1[i,1] - pos1[i-1,1]) < 1e-6): # current point should be different from the previous one
        # if current point is the same as the previous one go to next iteration
        continue
    else:
        # compute delta vector between two samples of g-code
        dp1 = np.array([[pos1[i,0] - pos1[i-1, 0], pos1[i,1] - pos1[i-1, 1]]]) 
        dp2 = np.array([[pos2[i,0] - pos2[i-1, 0], pos2[i,1] - pos2[i-1, 1]]])
        # compute length of these delta vectors
        dp1_len = norm(dp1, 2)
        dp2_len = norm(dp2, 2) 
        # number of section in between original points
        nr_sect = int(round(dp1_len/(c_speed*st))) 
        #print(len(p1x), "+ ", nr_sect)

        # append everthing to lists       
        for j in range(nr_sect):
            # tcp1
            p1x.append(pos1[i-1,0] + j * dp1[0,0] * 1/nr_sect) 
            p1y.append(pos1[i-1,1] + j * dp1[0,1] * 1/nr_sect)
            # tcp2
            p2x.append(pos2[i-1,0] + j * dp2[0,0] * 1/nr_sect) 
            p2y.append(pos2[i-1,1] + j* dp2[0,1] * 1/nr_sect)
    

# TODO: plot and test for different cutting speeds and check functionality
p1 = np.hstack((np.array(p1x).reshape(len(p1x),1), np.array(p1y).reshape(len(p1y),1)))
p2 = np.hstack((np.array(p2x).reshape(len(p2x),1), np.array(p2y).reshape(len(p2y),1)))

p_next = p1[1:-1, :]
p_is   = p1[0:-2, :]
v1 = list(map(lambda x, y: (x-y)/st, p_next, p_is))
# add additional velocity entries at start and end to match others arrays lengths
v1.insert(0, v1[0])
v1.append(v1[-1])
v1 = np.array(v1)
v1 = np.hstack((v1, np.zeros((len(v1[:,0]), 1))))


# do plots for tcp1
fig = plt.figure()
plt.scatter(p1[:,0], p1[:,1])
plt.scatter(pos1[:,0], pos1[:,1], marker="x")
plt.show()

fig = plt.figure()
v1_abs = np.array(list(map(lambda x: norm(x, 2), v1))).reshape(len(v1[:,0]),1) 
plt.scatter(np.linspace(0, len(v1[:,0])-1, len(v1[:,0])).reshape(len(v1[:,0]),1), v1_abs, marker='x')
plt.show()

# make a 3D visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(p1[:,0], p1[:,1], np.zeros(len(p1[:,0])))
ax.scatter(pos1[:,0], pos1[:,1], np.zeros(len(pos1[:,0])), marker="x")
ax.plot(p1[:,0], p1[:,1], np.squeeze(v1_abs))
plt.show()

# do plot for tcp2
fig = plt.figure()
plt.scatter(p2[:,0], p2[:,1])
plt.scatter(pos2[:,0], pos2[:,1], marker="x")
plt.show()


# number of points in paths
pNum = len(p1[:,0])
# length of wire, defined in 4-axis setup 
wLen = 0.8 
z1 = np.zeros((pNum,1))
z2 = np.ones((pNum,1)) * wLen
# append z axis to position vectors
p1m = np.hstack((p1,z1))
p2m = np.hstack((p1,z2))
p2m_ref = np.hstack((p2,z2)) # original coordinates
# define array to compute distance between effectors
effLen = np.zeros((pNum,1))
# using the zyx euler convention to store orientation - same in robot studio
ang = np.zeros((pNum,3))

for i in range(pNum):
    x1, x2 = p1[i,0], p2[i,0]
    y1, y2 = p1[i,1], p2[i,1]
    dx = x2 - x1
    dy = y2 - y1
    # compute angles based on geometric evaluation of setup, look at it as cuboid with wire from lower, left, back corner
    # to  upper, right, front corner. Calculate length of wire and angles using basic geometrics. 
    hyp1 = sqrt(dx**2 + wLen**2) # calculate first hypothenuse
    hyp2 = sqrt(dy**2 + hyp1**2) 
    ang[i, 2] = np.arctan(dx/wLen) # rotation around y-axis from oringal frame
    ang[i, 1] = np.arctan(dy/hyp1) # rotation around x-axis in already rotated frame
    effLen[i, 0] = hyp2 # effective length
    dLen = hyp2 - wLen 
    
    # compute rotation matrices
    # rotation in negative direction
    Rx = np.array([[1, 0, 0], 
              [0, cos(ang[i,1]), sin(ang[i,1])],
              [0, -sin(ang[i,1]), cos(ang[i,1])]])

    # rotation in postive direction
    Ry = np.array([[cos(ang[i,2]), 0, sin(ang[i,2])], 
              [0, 1, 0],
              [-sin(ang[i,2]), 0, cos(ang[i,2])]])

    # rotation matrix
    # self defined convention: rotate around y axis, then around x axis the get from  initial frame to wire frame
    # rotation from initial to first frame (z-axis aligned with wire)
    R01 = Ry @ Rx
    # rotation from first frame to initial frame
    R10 = np.transpose(R01)

    # build transformation matrix (rotation and translation) - r02 = T01 x r12
    # vector 1 in inital frame
    r0_01 = np.array([[x1], [y1], [0]])
    # vector in wires direction in first frame
    r1_12 = np.array([[0], [0], [wLen]])
    r0_12 = R01 @ r1_12
    r0_02 = r0_12 + r0_01

    # store values, cut last entry being 1 that was added for the transformation
    p2m[i,:] = np.transpose(r0_02)

# compute the respective vecolity of p2 - after doing the interpolation it takes the sample time st to the next point
# slice the first and the last sample 
p_next = p2m[1:-1, :]
p_is   = p2m[0:-2, :]
v2 = list(map(lambda x, y: (x-y)/st, p_next, p_is))
# add additional velocity entries at start and end to match others arrays lengths
v2.insert(0, v2[0])
v2.append(v2[-1])
# list to array conversion
v2 = np.array(v2)

# to the same for the orientation
o_next = ang[1:-1, :]
o_is   = ang[0:-2, :]
odot = list(map(lambda x, y: (x-y)/st, o_next, o_is))
# add additional velocity entries at start and end to match others arrays lengths
odot.insert(0, odot[0])
odot.append(odot[-1])
# list to array conversion
odot = np.array(odot)

# do some tests that check for correctness
# check if distance between two final points matches wires length
r_12 = p2m - p1m
dist_12 = list(map(lambda x: norm(x, 2), r_12))
# check if distance between computed second point and original one + wire's length matches effective length
r_12_ = p2m_ref - p2m
dist_22_ = list(map(lambda x: norm(x, 2), r_12_))

for i, (r12,r22_) in enumerate(zip(dist_12, dist_22_)):
    assert np.abs(r12 - wLen) < 0.00001, "path coordinate of p2 violates wire's constraint"
    assert (effLen[i, 0] - wLen - r22_) < 0.00001, "path coordinate of p2 violates against to be compensated length difference"


# plot the data for visualisation
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.scatter(p1m[:,0],p1m[:,1],p1m[:,2])
ax.scatter(p2m[:,0],p2m[:,1],p2m[:,2])
ax.scatter(p2m_ref[:,0],p2m_ref[:,1],p2m_ref[:,2])
ax.set_xlim(0,0.170)
ax.set_ylim(0,0.100)
ax.set_zlim(0.795,0.805)
plt.show()

# make data ready for export
traj_data = np.hstack((p1m, v1, p2m, v2, ang, odot))
np.save('traj_data', traj_data)