from cgitb import small
from operator import index
from unicodedata import decimal
import matplotlib.cm as cmx
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import matplotlib

import numpy as np

import_path = '/home/joschua/Coding/forceControl/master-project/python/plots/taskSpacePlacement/'
myX_L = np.load(import_path+'myX_L_nullgradient.npy')
myY_L = np.load(import_path+'myY_L_nullgradient.npy')
myZ_L = np.load(import_path+'myZ_L_nullgradient.npy')
smallestMan = np.load(import_path+'smallestMan_nullgradient.npy')

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

transparancy = np.ones((len(myX_L), 4))
dsmallestMan = (max(smallestMan)-min(smallestMan))
transparancy[:, 3] = np.array(smallestMan * (1/dsmallestMan) * 0.8 + 0.2)

man_list = smallestMan.tolist()
indexMax = man_list.index(max(man_list))
print("biggest manipulability coordintes xyz ")
print(str(myX_L.tolist()[indexMax]) + '\t' + str(myY_L.tolist()[indexMax]) + '\t'+ str(myZ_L.tolist()[indexMax]))


def scatter3d(x,y,z, cs, colorsMap='RdBu'):
    cm = plt.get_cmap(colorsMap)
    cNorm = matplotlib.colors.Normalize(vmin=min(cs), vmax=max(cs))
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cm)
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(x, y, z, c=np.round_(scalarMap.to_rgba(cs)*transparancy, decimals=4), depthshade=False)
    ax.set_ylim(50, 10)
    ax.set_xlabel('$y$ in cm')
    ax.set_ylabel('$x$ in cm')
    ax.set_zlabel('$z$ in cm')
    #ax.scatter(x, y, z, c=scalarMap.to_rgba(cs))
    scalarMap.set_array(cs)
    fig.colorbar(scalarMap)
    #fig.tight_layout()
    ax.view_init(elev=9., azim=-86.)
    plt.yticks([10, 20, 30, 40, 50])
    
    plt.show()

scatter3d(np.array(myY_L)*100, np.array(myX_L)*100, np.array(myZ_L)*100, np.array(smallestMan))