from cProfile import label
import numpy as np
import matplotlib.pyplot as plt


try:
    data = np.load('data/experimentLogs.npy')
    data2 = np.load('data/forceSignal-500.npy')
except FileNotFoundError:
    print('It seems like you need to run the script first that generate the requested data')

# p2, phi_delta, log_compPose_R, log_realPose_R, log_compJoints_R, log_realJoints_R, force
desPose = data[:, 0:6]
compPose = data[:, 6:12]
realPose = data[:, 12:18]
compJoints = data[:, 8:25]
realJoints = data[2:, 5:32]
force = data[:, 32:33]
noSamples = len(force)

time = np.linspace(0, round(1.0/80.0 * noSamples), num=noSamples)
time2 = np.linspace(0, round(1.0/80.0 * len(data2)), num=len(data2))
#time.shape = (time.size//1, 1)



fig = plt.figure()

ax1 = fig.add_subplot(211)
ax1.plot(time, force[:, 0]*5)
ax1.set_title('force trajectory')
ax1.set_ylabel('N')
ax1.set_xlabel('s')

ax2 = fig.add_subplot(212)
ax2.plot(time2, data2)
ax2.set_title('force constant weight')
ax2.set_ylabel('N')
ax2.set_xlabel('s')




""" ax2 = fig.add_subplot(122)
ax2.plot(time, compJoints[:, 0], label="J1")
ax2.plot(time, compJoints[:, 1], label="J2")
ax2.plot(time, compJoints[:, 2], label="J3")
ax2.plot(time, compJoints[:, 3], label="J4")

ax2.plot(time[0:-2], realJoints[:, 0], label="J1", linestyle="dashed")
ax2.plot(time[0:-2], realJoints[:, 1], label="J2", linestyle="dashed")
ax2.plot(time[0:-2], realJoints[:, 2], label="J3", linestyle="dashed")
ax2.plot(time[0:-2], realJoints[:, 3], label="J4", linestyle="dashed")
ax2.set_title('joint angles')
ax2.set_ylabel('rad')
ax2.set_xlabel('s')
ax2.legend() """


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(desPose[1:-1, 0], desPose[1:-1, 1], desPose[1:-1, 2], label="desPose")
ax.plot(compPose[1:-1, 0], compPose[1:-1, 1], compPose[1:-1, 2], label="compPose")
ax.plot(realPose[1:-1, 0], realPose[1:-1, 1], realPose[1:-1, 2], label="realPose")
ax.set_ylabel('y')
ax.set_xlabel('x')
ax.set_zlabel('z')
ax.set_title('trajectory right arm')
ax.legend()



 





""" axs[1, 0].set_title('Axis [1, 0]')
axs[1, 1].plot(x, -y, 'tab:red')
axs[1, 1].set_title('Axis [1, 1]')  """
plt.show()