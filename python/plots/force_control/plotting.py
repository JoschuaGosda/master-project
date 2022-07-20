import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from numpy.linalg import norm


R_01_80 = np.load('/home/joschua/Coding/forceControl/master-project/python/plots/postprocessing/R_01.npy')
R_01_250 = np.load('/home/joschua/Coding/forceControl/master-project/python/plots/postprocessing/R_01_250.npy')

data01 = np.load('./plots/force_control/Kp0.1-Op4.npy') # o for original
force01 = data01[:, 32:33]-4

data015 = np.load('./plots/force_control/Kp0.15-Op4.npy') # o for original
force015 = data015[:, 32:33]-4

datanof = np.load('./plots/force_control/noforceControl.npy') # o for original
forcenof = datanof[:, 32:33]-4

# dataabb = np.load('./plots/force_control/refKp0.05-Op4_v2.npy') # o for original
# p2_abb = dataabb[:, 63:66]
# p1_abb = dataabb[:, 66:69]

data_newUrdf = np.load('./plots/force_control/refKp0.05-Op4_v3.npy') # o for original
p1_is_new = data_newUrdf[:, 43:46]* 1000 
p2_is_new = data_newUrdf[:, 12:15]* 1000 
p2_abb = data_newUrdf[:, 63:66]
p1_abb = data_newUrdf[:, 66:69]


#experimentLogs = np.hstack((p2, phi_delta, log_compPose_R, log_realPose_R, log_compJoints_R, log_realJoints_R, log_force, p1, log_compJoints_R, log_realPose_L, log_compJoints_L, log_realJoints_L))
datao = np.load('./plots/force_control/Kp0.05-Op4.npy') # o for original
force = datao[:, 32:33]
p1_des = datao[:, 33:36]* 1000 
p2_des = datao[:, 0:3]* 1000 
p1_is = datao[:, 43:46]* 1000 
p2_is = datao[:, 12:15]* 1000 
e_f = force - 4.0
e_p = p2_des - p2_is

noSamples80 = len(e_p)  


# transform error into eef80frame
for i in range(noSamples80):
    e_p[i, :] = np.transpose(R_01_80[i] @ e_p[i, :].T) 


time80 = np.linspace(0, round(1.0/80.0 * noSamples80), num=noSamples80)

distance = norm(p1_is_new - p2_is_new, axis=1)
distance_abb = norm(p1_abb - p2_abb, axis=1)


y = e_f[10:1000]

n = len(y) # length of the signal
k = np.arange(n)
T = n/80
frq = k/T # two sides frequency range
frq = frq[:len(frq)//2] # one side frequency range

Y = np.fft.fft(y)/n # dft and normalization
Y = Y[:n//2]

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

# error plot
fig = plt.figure(figsize=(6.25, 2))

ax1 = fig.add_subplot(111)
ax1.plot(time80[0:-10] , e_p [0:-10, 1], label='position error')
ax1.set_ylabel('$y_{EE, R}$ in mm')
ax1.set_xlabel('time in s')
 
ax4 = ax1.twinx() 
ax4.plot(time80[0:-10] , e_f[0:-10] , color=next(ax1._get_lines.prop_cycler)['color'], label='force')  
ax4.set_ylim(-1, 1) 
ax4.set_ylabel('force in N')
labels = ["position error", "force error"]
fig.legend( labels=labels,
            loc='upper right', bbox_to_anchor=(0.6, 0.92),
            ncol=1)

fig.tight_layout()
#fig.subplots_adjust(top=0.9)
plt.show()  



fig = plt.figure(figsize=(6.25, 2))
ax2 = fig.add_subplot(111)
#ax1.set_title('position error in EE frame')
ax2.plot(time80[0:-10] , distance [0:-10], label='comp. pose')
ax2.plot(time80[0:-10] , distance_abb [0:-10], label='meas. pose')
ax2.set_ylabel('$l^2$-norm in mm')
plt.yticks([402, 402.5, 403, 403.5, 404])
ax2.set_xlabel('time in s')
ax2.legend()
fig.tight_layout()
#plt.title('hybrid control at Kp=0.05 and OP=4N')
#fig.subplots_adjust(top=0.9)
plt.show()  



fig = plt.figure()

ax3 = fig.add_subplot(111)
ax3.plot(frq, abs(Y), color=next(ax2._get_lines.prop_cycler)['color'], label='force')
ax3.set_xlabel('freq (Hz)')
ax3.set_ylabel('|Y(freq)|')
ax3.legend()



#labels = ["case 1", "case 2", "case 3", "case 4"]
#fig.legend( labels=labels,
#           loc="upper center", ncol=4)
#fig.subplots_adjust(top=0.9, hspace=0.3)
fig.tight_layout()
#fig.subplots_adjust(top=0.9)
plt.show()  


# make a 3D visualization
# right
fig = plt.figure(figsize=(3, 3))
ax = fig.add_subplot(111, projection='3d')
ax.plot(p2_des[0:-10,0], p2_des[0:-10,1], p2_des[0:-10,2], label='des. pose')
#ax.plot(p2_is[0:-10,0], p2_is[0:-10,1], p2_is[0:-10,2], label='real_pathR')
ax.plot(p2_is_new[0:-10,0], p2_is_new[0:-10,1], p2_is_new[0:-10,2], label='comp. pose')
ax.plot(p2_abb[0:-10,0], p2_abb[0:-10,1], p2_abb[0:-10,2], label='meas. pose')

#ax.set_xlim(54, 66)
#ax.set_ylim(99, 101)
#ax.set_xlabel('$x$ in mm')
ax.set_ylabel('$y$ in mm')
ax.set_zlabel('$z$ in mm')
ax.view_init(elev=33., azim=0.)
ax.legend(loc='upper right')
#fig.tight_layout()
plt.show() 

#left
fig = plt.figure(figsize=(3, 3))
ax2 = fig.add_subplot(111, projection='3d')
ax2.plot(p1_des[0:-10,0], p1_des[0:-10,1], p1_des[0:-10,2], label='des. pose')
#ax2.plot(p1_is[0:-10,0], p1_is[0:-10,1], p1_is[0:-10,2], label='real_pathR')
ax2.plot(p1_is_new[0:-10,0], p1_is_new[0:-10,1], p1_is_new[0:-10,2], label='comp. pose')
ax2.plot(p1_abb[0:-10,0], p1_abb[0:-10,1], p1_abb[0:-10,2], label='meas. pose')
ax2.set_xlabel('$x$ in mm')
ax2.set_ylabel('$y$ in mm')
#ax2.set_zlabel('$z$ in mm')
ax2.set_ylim(98, 101)
ax2.view_init(elev=33., azim=0.)
ax2.legend(loc='upper right')
plt.yticks([98, 99, 100, 101])
#fig.tight_layout()
plt.show() 


fig = plt.figure()

ax1 = fig.add_subplot(111)
ax1.plot(time80[0:-10], force015[0:-10], label='0.15')
ax1.plot(time80[0:-10], force01[0:-10], label='0.1')
ax1.plot(time80[0:-10], e_f[0:-10], label='0.05')
ax1.plot(time80[0:-10], forcenof[0:-10], label='pure position')

ax1.set_ylabel('N')
ax1.set_xlabel('s')
ax1.legend()
plt.title('force errors in comparision')
fig.tight_layout()
plt.show()



