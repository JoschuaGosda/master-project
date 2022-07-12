import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Rectangle
from numpy.linalg import norm

import_path = '/home/joschua/Coding/forceControl/master-project/python/plots/preprocessing/'
p1 = np.load(import_path+'p1.npy')
p2 = np.load(import_path+'p2.npy')
p1m = np.load(import_path+'p1m.npy')
p2m = np.load(import_path+'p2m.npy')
pos1 = np.load(import_path+'pos1.npy')
pos2 = np.load(import_path+'pos2.npy')
p2m_ref = np.load(import_path+'p2m_ref.npy')
v1 = np.load(import_path+'v1.npy')
v2 = np.load(import_path+'v2.npy')

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

""" # do plots for tcp1
#fig = plt.figure()
fig = plt.figure(figsize=(6.25, 2))
ax = fig.add_subplot(111)
ax.scatter(p1[:,0]*1000, p1[:,1]*1000-53.7, label='$p_{1*}$', color='#005293')
ax.scatter(pos1[:,0]*1000, pos1[:,1]*1000-53.7, marker="x", label='$p_1$', color='#80a9c9')
# part view
ax.set_xlim(10.8, 14.8)
ax.set_ylim(-4.4, -1.95)
#  overview
#ax.set_xlim(-10, 175)
#ax.set_ylim(-10, 25)
ax.set_xlabel('$x_1$')
ax.set_ylabel('$y_1$')
ax.legend()
#ax.add_patch(Rectangle((10.8, -4.4), 4, 2.45,
#             edgecolor = 'black',
#             #facecolor = 'blue',
#             fill=False,
#             lw=1))
plt.show()

fig.tight_layout()
#fig.savefig('/home/joschua/Documents/Studium/TUM/Thesis/documentation/thesis/myWorkFiles/AMStudentThesis/figures/plots/lefttraj_part.pgf', format='pgf')
 """

fig = plt.figure(figsize=(6.25, 2))
ax = fig.add_subplot(111)
v1_abs = np.array(list(map(lambda x: norm(x, 2), v1))).reshape(len(v1[:,0]),1) 
v2_abs = np.array(list(map(lambda x: norm(x, 2), v2))).reshape(len(v2[:,0]),1) 
timeline = np.linspace(0, (len(v1[:,0])-1)*1.0/80.0, len(v1[:,0])).reshape(len(v1[:,0]),1)
v_des = np.ones((len(timeline), 1))* 1000
ax.plot( timeline, v1_abs*1000*60, label='$v_1$')
ax.plot( timeline, v2_abs*1000*60, label='$v_2$')
ax.plot( timeline, v_des, linestyle='dashed', label='$v_f$')
ax.set_xlabel('s')
ax.set_ylabel('mm/min')
ax.legend()
#plt.ylim(bottom=0)
fig.tight_layout()
plt.show()


""" 
# make a 3D visualization
fig = plt.figure(figsize=(10, 5))
ax = fig.add_subplot(111, projection='3d')
ax.plot(p1[:,0]*1000, p1[:,1]*1000, np.zeros(len(p1[:,0])))
#ax.scatter(pos1[:,0], pos1[:,1], np.zeros(len(pos1[:,0])), marker="x")
ax.plot(p1[:,0]*1000, p1[:,1]*1000, np.squeeze(v1_abs)* 1000)
ax.set_xlim(54, 66)
ax.set_ylim(0, 100)
ax.view_init(elev=25., azim=-115.)
plt.show() 

# do plot for tcp2
fig = plt.figure()
plt.scatter(p2[:,0], p2[:,1])
plt.scatter(pos2[:,0], pos2[:,1], marker="x")
plt.show() 


fig = plt.figure()
plt.scatter(p1m[200:300, 0], p1m[200:300, 2], label="p1")
plt.scatter(p2m_ref[200:300, 0], p2m_ref[200:300,2], label="p2_ref")
plt.scatter(p2m[200:300, 0], p2m[200:300,2], label="p2")
plt.legend()
plt.show()



# plot the data for visualisation
fig = plt.figure(figsize=(3, 3))
ax = fig.add_subplot(111, projection='3d')
#ax.plot(p1m[:,0]*1000, p1m[:,2]*1000, p1m[:,1]*1000-53.7, color='#005293', label='$p_{1*}$')
ax.plot(p2m_ref[:,0]*1000,p2m_ref[:,2]*1000, p2m_ref[:,1]*1000-53.7, label='$p_2$')
ax.plot(p2m[:,0]*1000,p2m[:,2]*1000, p2m[:,1]*1000-53.7, label='$p_{2*}$')
#ax.set_xlim(-10,170)
#ax.set_ylim(-5, 5)#axes were switched to better rotate plot is z
ax.set_ylim(405, 395)#axes were switched to better rotate plot is z
ax.set_zlim(-50,15)#axes were switched to better rotate plot is y
#ax.set_zlim(-15,25)
ax.view_init(elev=13., azim=36.)
ax.set_xlabel('$x_2$')
ax.set_ylabel('$z_2$')
ax.set_zlabel('$y_2$')
ax.legend()
plt.show() 

fig.tight_layout()  """