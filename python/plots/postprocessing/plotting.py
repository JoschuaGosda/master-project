import numpy as np
import matplotlib.pyplot as plt


R_01 = np.load('/home/joschua/Coding/forceControl/master-project/python/plots/postprocessing/R_01.npy')

#experimentLogs = np.hstack((p2, phi_delta, log_compPose_R, log_realPose_R, log_compJoints_R, log_realJoints_R, log_force, p1, log_compJoints_R, log_realPose_L, log_compJoints_L, log_realJoints_L))

data = np.load('./data/experimentLogs300-0,4-1.npy')
p2_des = data[:, 0:3]
p2_is = data[:, 12:15]
p2_comp = data[:, 6:9]
force = data[:, 32:33]*5-5

# error in workspace
e = (p2_des - p2_is) * 1000 # m 2 mm
e_ik = (p2_des - p2_comp) * 1000 # m 2 mm

noSamples = len(e)  

# transform error into ee frame
for i in range(noSamples):
    e[i, :] = np.transpose(R_01[i] @ e[i, :].T)
    e_ik[i, :] = np.transpose(R_01[i] @ e_ik[i, :].T)




time = np.linspace(0, round(1.0/80.0 * noSamples), num=noSamples)



fig = plt.figure()

ax1 = fig.add_subplot(311)
ax2 = fig.add_subplot(312)
ax3 = fig.add_subplot(313)


ax1.plot(time[0:-10], e[0:-10, 0], label='e_x')
#ax.plot(time[0:-10], e_ik[0:-10, 0], label='eik_x_ee')
ax2.plot(time[0:-10], e[0:-10, 1]*(-1), label='e_y')
ax3.plot(time[0:-10], e[0:-10, 2], label='e_z')

ax4 = ax2.twinx() 
ax4.plot(time[0:-10], force[0:-10], label='force', color='r')  
ax4.set_ylim(-1, 1) 
ax4.set_ylabel('N')

ax1.set_title('position error in EE frame')
ax1.set_ylabel('mm')
ax1.set_xlabel('s')
ax2.set_ylabel('mm')
ax2.set_xlabel('s')
ax3.set_ylabel('mm')
ax3.set_xlabel('s')
ax1.legend()
ax2.legend()
ax3.legend()
plt.show()