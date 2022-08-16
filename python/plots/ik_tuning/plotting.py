from turtle import color
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Rectangle
from numpy.linalg import norm

import_path = '/home/joschua/Coding/forceControl/master-project/python/plots/ik_tuning/'
error_right = np.load(import_path+'error_right.npy')
#el00 = np.load(import_path+'error_left0.npy')
#el01 = np.load(import_path+'error_left0.1.npy')
error_left = np.load(import_path+'error_left1.npy')
#el2 = np.load(import_path+'error_left2.npy')
#el5 = np.load(import_path+'error_left5.npy')
#error_left0 = np.load(import_path+'error_left10.npy')
computedPose_left = np.load(import_path+'computedPose_left.npy')
computedPose_right= np.load(import_path+'computedPose_right.npy')
p1 = np.load(import_path+'p1.npy')
p2 = np.load(import_path+'p2.npy')
compJointAngles_right = np.load(import_path+'compJointAngles_right.npy')
compJointAngles_left = np.load(import_path+'compJointAngles_left.npy')
jointVelociies_left = np.load(import_path+'jointVelocities_left.npy')
jointVelociies_right = np.load(import_path+'jointVelocities_right.npy')

noSamples = len(jointVelociies_left)
time = np.linspace(0, round(1.0/80.0 * noSamples), num=noSamples)

# use self-defined tum-cycler for TUM-blue colors
plt.style.use('mylatex')

# plotting for thesis
plt.rcParams.update({
    "font.family": "serif",  # use serif/main font for text elements
    "text.usetex": True,     # use inline math for ticks
    "pgf.rcfonts": False,     # don't setup fonts from rc parameters
    #"legend.loc": 'upper right',
    "savefig.directory": '/home/joschua/Documents/Studium/TUM/Thesis/documentation/thesis/myWorkFiles/AMStudentThesis/figures/plots/'
    })

for i in range(len(error_left)):
    if error_left[i,3] < -np.pi/2:
        error_left[i,3] = error_left[i,3] + np.pi
    elif error_left[i,3] > np.pi/2:
        error_left[i,3] = error_left[i,3] - np.pi
    if error_left[i,4] < -np.pi/2:
        error_left[i,4] = error_left[i,4] + np.pi
    elif error_left[i,4] > np.pi/2:
        error_left[i,4] = error_left[i,4] - np.pi
    if error_left[i,5] < -np.pi/2:
        error_left[i,5] = error_left[i,5] + np.pi
    elif error_left[i,5] > np.pi/2:
        error_left[i,5] = error_left[i,5] - np.pi

""" fig = plt.figure()
plt.plot(error_left[10:-1, 3:6])
plt.show()
 """
print('biggest translational error norm' + str(max(np.linalg.norm(error_left[10:-1,0:3]*1000, axis=1))))
print('biggest rotational error x' + str(max(error_left[10:-1,3])))
print('biggest rotational error y' + str(max(error_left[10:-1,4])))
print('biggest rotational error z' + str(max(error_left[10:-1,5])))

# see development of joint values
""" fig = plt.figure()
plt.plot(compJointAngles_left[:,0], label='joint1')
plt.plot(compJointAngles_left[:,1], label='joint2')
plt.plot(compJointAngles_left[:,2], label='joint3')
plt.plot(compJointAngles_left[:,3], label='joint4')
plt.plot(compJointAngles_left[:,4], label='joint5')
plt.plot(compJointAngles_left[:,5], label='joint6')
plt.plot(compJointAngles_left[:,6], label='joint7')
plt.title('joint values over samples, left arm')
plt.legend()
plt.show() """

""" base_error = np.array((el00[0]))

error_left = np.concatenate(([base_error], error_left), axis=0)
el01 = np.concatenate(([base_error], el01), axis=0)
el00 = np.concatenate(([base_error], el00), axis=0) """


""" # error
fig = plt.figure(figsize=(6.25, 2))
ax = fig.add_subplot(111)
#ax.plot(time, np.linalg.norm(el00[:,0:3]*1000, axis=1), label='$0.0$')
#ax.plot(time, np.linalg.norm(el01[:,0:3]*1000, axis=1), label='$0.1$')
#ax.plot(time, np.linalg.norm(error_left[:,0:3]*1000, axis=1), label='$1.0$', color='#333333')
ax.plot(np.linalg.norm(error_left[:,0:3]*1000, axis=1), label='$1.0$', color='#333333')
ax.set_ylabel('mm')
ax.set_xlabel('s')
#plt.plot(el2[:,0], label='el2')
#plt.plot(el5[:,0], label='el5')


#plt.plot(error_left[:,1], label='y')
#plt.plot(error_left[:,2], label='z')
#plt.plot(error_left[:,3], label='rx')
#plt.plot(error_left[:,4], label='ry')
#plt.plot(error_left[:,5], label='rz')
plt.legend()
fig.tight_layout()
plt.show() """

""" # show trajectory in workspace of yumi
fig = plt.figure()
plt.plot(p1[:,0], p1[:,2], label='desired profile') # plot z over x
plt.scatter(computedPose_left[:,0], computedPose_left[:,2], label='resulting pose from IK')
fig.get_axes()[0].set_xlabel('x axis of yumi')
fig.get_axes()[0].set_ylabel('z axis of yumi')
plt.legend()
plt.title('poses left')
plt.show()  """

""" fig = plt.figure()
plt.plot(computedPose_left[:,3], label='rx')
plt.plot(computedPose_left[:,4], label='ry')
plt.plot(computedPose_left[:,5], label='rz')
plt.legend()
plt.title('euler angles over trajectories')
plt.show() """

""" fig = plt.figure()
plt.plot(compJointAngles_right[:,0], label='joint1')
plt.plot(compJointAngles_right[:,1], label='joint2')
plt.plot(compJointAngles_right[:,2], label='joint3')
plt.plot(compJointAngles_right[:,3], label='joint4')
plt.plot(compJointAngles_right[:,4], label='joint5')
plt.plot(compJointAngles_right[:,5], label='joint6')
plt.plot(compJointAngles_right[:,6], label='joint7')
plt.title('joint values over samples, right arm')
plt.legend()
plt.show() """

""" fig = plt.figure()
plt.plot(error_right[:,0], label='x')
plt.plot(error_right[:,1], label='y')
plt.plot(error_right[:,2], label='z')
#plt.plot(error_right[:,3], label='rx')
#plt.plot(error_right[:,4], label='ry')
#plt.plot(error_right[:,5], label='rz')
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
plt.show() """


fig = plt.figure(figsize=(6.25, 5))

ax1 = fig.add_subplot(221)
ax2 = fig.add_subplot(223)
ax3 = fig.add_subplot(222)
ax4 = fig.add_subplot(224)

ax1.plot(time[10:-1], compJointAngles_right[10:-1,0], label='joint1')
ax1.plot(time[10:-1], compJointAngles_right[10:-1,1], label='joint2')
ax1.plot(time[10:-1], compJointAngles_right[10:-1,2], label='joint3')
ax1.plot(time[10:-1], compJointAngles_right[10:-1,3], label='joint4')
ax1.plot(time[10:-1], compJointAngles_right[10:-1,4], label='joint5')
ax1.plot(time[10:-1], compJointAngles_right[10:-1,5], label='joint6')
ax1.plot(time[10:-1], compJointAngles_right[10:-1,6], label='joint7')

ax2.plot(time[10:-1], jointVelociies_right[10:-1,0], label='joint1')
ax2.plot(time[10:-1], jointVelociies_right[10:-1,1], label='joint2')
ax2.plot(time[10:-1], jointVelociies_right[10:-1,2], label='joint3')
ax2.plot(time[10:-1], jointVelociies_right[10:-1,3], label='joint4')
ax2.plot(time[10:-1], jointVelociies_right[10:-1,4], label='joint5')
ax2.plot(time[10:-1], jointVelociies_right[10:-1,5], label='joint6')
ax2.plot(time[10:-1], jointVelociies_right[10:-1,6], label='joint7')

ax3.plot(time[10:-1], compJointAngles_left[10:-1,0], label='joint1')
ax3.plot(time[10:-1], compJointAngles_left[10:-1,1], label='joint2')
ax3.plot(time[10:-1], compJointAngles_left[10:-1,2], label='joint3')
ax3.plot(time[10:-1], compJointAngles_left[10:-1,3], label='joint4')
ax3.plot(time[10:-1], compJointAngles_left[10:-1,4], label='joint5')
ax3.plot(time[10:-1], compJointAngles_left[10:-1,5], label='joint6')
ax3.plot(time[10:-1], compJointAngles_left[10:-1,6], label='joint7')

ax4.plot(time[10:-1], jointVelociies_left[10:-1,0], label='joint1')
ax4.plot(time[10:-1], jointVelociies_left[10:-1,1], label='joint2')
ax4.plot(time[10:-1], jointVelociies_left[10:-1,2], label='joint3')
ax4.plot(time[10:-1], jointVelociies_left[10:-1,3], label='joint4')
ax4.plot(time[10:-1], jointVelociies_left[10:-1,4], label='joint5')
ax4.plot(time[10:-1], jointVelociies_left[10:-1,5], label='joint6')
ax4.plot(time[10:-1], jointVelociies_left[10:-1,6], label='joint7')

ax1.set_title('')
ax1.set_ylabel('angle in rad')
#ax1.set_xlabel('s')
ax2.set_ylabel('velocity in rad/s')
ax2.set_xlabel('time in s')
#ax3.set_ylabel('rad')
#ax3.set_xlabel('s')
#ax4.set_ylabel('rad/s')
ax4.set_xlabel('time in s')
#ax1.legend()
#ax2.legend()
#ax3.legend()
#ax4.legend()
labels = ["joint 1", "joint 2", "joint 3", "joint 4", "joint 5", "joint 6", "joint 7",]
fig.legend( labels=labels,
           loc="upper center", ncol=4)
fig.subplots_adjust(top=0.9, hspace=0.3)
fig.tight_layout()
fig.subplots_adjust(top=0.85)
plt.show()