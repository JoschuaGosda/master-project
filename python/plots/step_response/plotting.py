import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec



''' Only 300-351015 and 300-351015-rawc20_v2 were saved in this format'''
#experimentLogs = np.hstack((p2, phi_delta, log_compPose_R, log_realPose_R, log_compJoints_R, log_realJoints_R, log_force, p1, log_compJoints_R, log_realPose_L, log_compJoints_L, log_realJoints_L))
datao = np.load('./plots/step_response/step_x.npy') # o for original
p1_des = datao[:, 33:36]* 1000 
p1_is = datao[:, 43:46]* 1000 
e = (p1_des - p1_is) 

#noSamples = len(p1_desf80)  
noSamples80 = len(p1_des)   

time80 = np.linspace(0, round(1.0/80.0 * noSamples80), num=noSamples80)

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
fig = plt.figure()

ax1 = fig.add_subplot(311)
ax2 = fig.add_subplot(312)
#ax3 = fig.add_subplot(313)

ax1.plot(time80 [30:-10], e     [30:-10, 0])
ax2.plot(time80 [30:-10], p1_des     [30:-10, 0])
ax2.plot(time80 [30:-10], p1_is     [30:-10, 0])


#ax1.set_title('position error in EE frame')
ax1.set_ylabel('$x$ in mm')
ax2.set_ylabel('$x$ in mm')
ax2.set_xlabel('s')

#labels = ["case 1", "case 2", "case 3", "case 4"]
#fig.legend( labels=labels,
#           loc="upper center", ncol=4)
#fig.subplots_adjust(top=0.9, hspace=0.3)
fig.tight_layout()
#fig.subplots_adjust(top=0.9)
plt.show()  
