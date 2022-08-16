import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Rectangle
import pandas as pd


R_01_80 = np.load('/home/joschua/Coding/forceControl/master-project/python/plots/postprocessing/R_01.npy')
R_01_250 = np.load('/home/joschua/Coding/forceControl/master-project/python/plots/postprocessing/R_01_250.npy')
R_01_600 = np.load('/home/joschua/Coding/forceControl/master-project/python/plots/postprocessing/R_01_600.npy')
R_01_900 = np.load('/home/joschua/Coding/forceControl/master-project/python/plots/postprocessing/R_01_900.npy')

''' Only 300-351015 and 300-351015-rawc20_v2 were saved in this format'''
#experimentLogs = np.hstack((p2, phi_delta, log_compPose_R, log_realPose_R, log_compJoints_R, log_realJoints_R, log_force, p1, log_compJoints_R, log_realPose_L, log_compJoints_L, log_realJoints_L))
datao = np.load('./plots/control_positionv2/v2-300-351015_default.npy') # o for original
p1_des = datao[:, 33:36]* 1000 
p1_is = datao[:, 43:46]* 1000 
#p2_des = datao[:, 0:3]* 1000
#p2_is = datao[:, 6:9]* 1000
e = (p1_des - p1_is) 

''' Measurements were saved in this format '''
#p2, phi_delta, log_realPose_R,  p1, log_realPose_L))
dataf80 = np.load('./plots/control_positionv2/v2-300-351015_L100.npy')
p1_desf80 = dataf80[:, 33:36]* 1000 
p1_isf80 = dataf80[:, 43:46]* 1000 
#p1_desf80 = dataf80[:, 0:3]* 1000
#p1_isf80 = dataf80[:, 6:9]* 1000 
# error in workspace
ef80 = (p1_desf80 - p1_isf80) 

datac20 = np.load('./plots/control_positionv2/v2-300-351015_P20.npy')
p1_desc20 = datac20[:, 33:36]* 1000 
p1_isc20 = datac20[:, 43:46]* 1000 
# error in workspace
ec20 = (p1_desc20 - p1_isc20) 

datatc250 = np.load('./plots/control_positionv2/v2-300-351015_C250.npy')
p1_destc250 = datatc250[:, 33:36]* 1000  
p1_istc250 = datatc250[:, 43:46]* 1000 
comptime = np.squeeze(datatc250[:, 69:70].reshape(1, len(datatc250)))*1000
comptime_smooth= pd.Series(comptime).rolling(window=250).mean()
# error in workspace
etc250 = (p1_destc250 - p1_istc250) 

datatc250_noM = np.load('./plots/control_positionv2/v2-300-351015_C250_noM.npy')
p1_destc250_noM = datatc250_noM[:, 33:36]* 1000 
p1_istc250no_M = datatc250_noM[:, 43:46]* 1000 
comptimeno_M = np.squeeze(datatc250_noM[:, 69:70].reshape(1, len(datatc250_noM)))
comptimeno_M_smooth= pd.Series(comptimeno_M).rolling(window=250).mean()
# error in workspace
etc250no_M = (p1_destc250_noM - p1_istc250no_M) 

data600 = np.load('./plots/control_positionv2/v2-300-351015_S600.npy')
p1_des600 = data600[:, 33:36]* 1000 
p1_is600 = data600[:, 43:46]* 1000 

# error in workspace
e600 = (p1_des600 - p1_is600) 

data900 = np.load('./plots/control_positionv2/v2-300-351015_S900.npy')
p1_des900 = data900[:, 33:36]* 1000 
p1_is900 = data900[:, 43:46]* 1000 

# error in workspace
e900 = (p1_des900 - p1_is900) 




#noSamples = len(p1_desf80)  
noSamples80 = len(ef80)  
noSamples250 = len(etc250) 
noSamples600 = len(e600) 
noSamples900 = len(e900) 



for i in range(noSamples900):
    e900[i, :] = np.transpose(R_01_900[i] @ e900[i, :].T) 

for i in range(noSamples600):
    e600[i, :] = np.transpose(R_01_600[i] @ e600[i, :].T) 

# transform error into eef80frame
for i in range(noSamples80):
    e[i, :] = np.transpose(R_01_80[i] @ e[i, :].T) 
    ef80[i, :] = np.transpose(R_01_80[i] @ ef80[i, :].T) 
    ec20[i, :] = np.transpose(R_01_80[i] @ ec20[i, :].T) 

for i in range(noSamples250):
    etc250[i, :] = np.transpose(R_01_250[i] @ etc250[i, :].T) 
    etc250no_M[i, :] = np.transpose(R_01_250[i] @ etc250no_M[i, :].T) 

time80 = np.linspace(0, round(1.0/80.0 * noSamples80), num=noSamples80)
time250 = np.linspace(0, round(1.0/250.0 * noSamples250), num=noSamples250)
time600 = np.linspace(0, round(1.0/80.0 * noSamples600), num=noSamples600)
time900 = np.linspace(0, round(1.0/80.0 * noSamples900), num=noSamples900)

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

fig = plt.figure(figsize=(6, 2))
ax1 = fig.add_subplot(111)

#ax1.plot(time250 [30:-10], comptimeno_M  [30:-10], label='off')
ax1.plot(time250 [30:-10], comptime     [30:-10], label='on')
ax1.set_ylabel('comp. time in ms')
ax1.set_xlabel('time in s')
fig.tight_layout()
#ax1.legend()
plt.show()
######################300
# error plot
fig = plt.figure(figsize=(6, 6.5))


ax1 = fig.add_subplot(411)
ax0 = fig.add_subplot(412)
ax2 = fig.add_subplot(413)
ax3 = fig.add_subplot(414)



ax1.plot(time80 [30:-10], e     [30:-10, 0])
#ax1.plot(time80 [30:-10], ec20  [30:-10, 0])
#ax1.plot(time80 [30:-10], ef80  [30:-10, 0])
ax1.plot(time250[30:-10], etc250[30:-10, 0])
#ax1.plot(time250[30:-10], etc250no_M[30:-10, 0])
ax1.set_ylabel('$x_{EE,L}$ in mm')
#ax1.add_patch(Rectangle((20, 0.78), 2.5, 0.34,
#             edgecolor = 'black',
#             fill=False,
#             zorder=2,
#             lw=1))

ax2.plot(time80 [30:-10], e     [30:-10, 1])
#ax2.plot(time80 [30:-10], ec20  [30:-10, 0])
#ax2.plot(time80 [30:-10], ef80  [30:-10, 0])
ax2.plot(time250[30:-10], etc250[30:-10, 1])
#ax2.plot(time250[30:-10], etc250no_M[30:-10, 0])
ax2.set_ylabel('$y_{EE,L}$ in mm')
#x2.set_xlim([20, 22.5])
ax2.set_ylim([-0.2, 0.2])


ax3.plot(time80 [30:-10], e     [30:-10, 2])
#ax3.plot(time80 [30:-10], ec20  [30:-10, 2])
#ax3.plot(time80 [30:-10], ef80  [30:-10, 2])
ax3.plot(time250[30:-10], etc250[30:-10, 2])
#ax3.plot(time250[30:-10], etc250no_M[30:-10, 2])

ax0.plot(time900[30:-10], e900[30:-10, 0],color=next(ax3._get_lines.prop_cycler)['color'])
ax0.set_ylabel('$x_{EE,L}$ in mm')

ax3.set_ylabel('$z_{EE,L}$ in mm')
ax3.set_xlabel('time in s')
#ax3.set_ylim([-1, 1])

#labels = ["Case 6", "Case 1, 2, 3", "Case 4", "Case 5"]
labels = [ "Case 1, 2, 3", "Case 4", "Case 5"]
fig.legend( labels=labels,
           loc="upper center", ncol=6)
#fig.subplots_adjust(top=0.9, hspace=0.3)
fig.tight_layout()
fig.subplots_adjust(top=0.945)
plt.show()  


# profile plot
""" fig = plt.figure() """
"""  """
""" ax1 = fig.add_subplot(311) """
""" ax2 = fig.add_subplot(312) """
""" ax3 = fig.add_subplot(323) """

fig = plt.figure(constrained_layout=True, figsize=(6, 4))
gs = GridSpec(2, 2, figure=fig)

ax1 = fig.add_subplot(gs[0, :])
ax2 = fig.add_subplot(gs[1, 0:1])
ax3 = fig.add_subplot(gs[1, 1:2])


ax1.plot(p1_is       [30:-10, 0], p1_is       [30:-10, 2], lw=4)
#ax1.plot(p1_isc20    [30:-10, 0], p1_isc20    [30:-10, 2])
#ax1.plot(p1_isf80    [30:-10, 0], p1_isf80    [30:-10, 2])
ax1.plot(p1_istc250 [30:-10, 0], p1_istc250 [30:-10, 2])
#ax1.plot(p1_istc250no_M [30:-10, 0], p1_istc250no_M [30:-10, 2])
ax1.plot(p1_is900       [30:-10, 0], p1_is900       [30:-10, 2])
ax1.plot(p1_des      [30:-10, 0], p1_des      [30:-10, 2], linestyle='dotted',  color='#808080')
ax1.add_patch(Rectangle((357.5, 146.5), 6.5, 6.5,
             edgecolor = 'black',
             #facecolor = 'blue',
             fill=False,
             lw=1))
ax1.add_patch(Rectangle((514.5, 150.5), 6.5, 6.5,
            edgecolor = 'black',
            #facecolor = 'blue',
            fill=False,
            lw=1))
ax1.set_xticks([360, 380, 420, 440, 460, 480, 500, 520])

ax2.plot(p1_is       [30:-10, 0], p1_is       [30:-10, 2], lw=4)
#ax2.plot(p1_isc20    [30:-10, 0], p1_isc20    [30:-10, 2])
#ax2.plot(p1_isf80    [30:-10, 0], p1_isf80    [30:-10, 2])
ax2.plot(p1_istc250 [30:-10, 0], p1_istc250 [30:-10, 2])
#ax2.plot(p1_istc250no_M [30:-10, 0], p1_istc250no_M [30:-10, 2])
ax2.plot(p1_is900       [30:-10, 0], p1_is900       [30:-10, 2])
ax2.plot(p1_des      [30:-10, 0], p1_des      [30:-10, 2], linestyle='dotted',  color='#808080')
ax2.set_ylim([146.5, 153])
ax2.set_xlim([357.5, 364])

ax3.plot(p1_is       [30:-10, 0], p1_is       [30:-10, 2], lw=4)
#ax3.plot(p1_isc20    [30:-10, 0], p1_isc20    [30:-10, 2])
#ax3.plot(p1_isf80    [30:-10, 0], p1_isf80    [30:-10, 2])
ax3.plot(p1_istc250 [30:-10, 0], p1_istc250 [30:-10, 2])
#ax3.plot(p1_istc250no_M [30:-10, 0], p1_istc250no_M [30:-10, 2])
ax3.plot(p1_is900       [30:-10, 0], p1_is900       [30:-10, 2])
ax3.plot(p1_des      [30:-10, 0], p1_des      [30:-10, 2], linestyle='dotted', color='#808080')
ax3.set_xlim([514.5, 521])
ax3.set_ylim([150.5, 157])

ax1.set_ylabel('$z$ in mm')
ax2.set_ylabel('$z$ in mm')
ax1.set_xlabel('$x$ in mm')
ax2.set_xlabel('$x$ in mm')
ax3.set_xlabel('$x$ in mm')

labels = ["Case 1, 2, 3", "Case 4", "Case 5", "reference"]
fig.legend( labels=labels,
           loc="upper center", ncol=5)
#fig.subplots_adjust(top=0.9, hspace=0.3)
fig.tight_layout()
fig.subplots_adjust(top=0.9)
plt.show() 

