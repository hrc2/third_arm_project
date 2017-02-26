import numpy as np
import math
import pylab
from matplotlib import colors as mcolors
from matplotlib.collections import LineCollection
from matplotlib.collections import PolyCollection
import matplotlib.pyplot as plt
import csv
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.tri as mtri

def read_data(fname):
    with open(fname, 'rb') as f:
        reader = csv.reader(f)
        # for row in reader:
        #     print row
        return list(reader)




v1_elbow = read_data('v1_elbow.csv')
v2_elbow = read_data('v2_elbow.csv')

v1_shoulder = read_data('v1_shoulder.csv')
v2_shoulder = read_data('v2_shoulder.csv')


fig1 = plt.figure()
fig1.suptitle('Moment Loads for Prototype Comparison',fontsize=20)
ax1 = fig1.add_subplot(111)
l1 = ax1.plot(v1_elbow,label='Model I Elbow',linewidth=1.0,ls='dashed',c='m')
l2 = ax1.plot(v2_elbow,label='Model II Elbow',linewidth=3.0,ls='dashed',c='b')

l3 = ax1.plot(v1_shoulder,label='Model I Shoulder',linewidth=1.0,c='k')
l4 = ax1.plot(v2_shoulder,label='Model II Shoulder',linewidth=3.0,c='r')
ax1.set_ylim([0,24])
ax1.set_xlabel('Time step',fontsize=14)
ax1.set_ylabel('Magnitude of moment (Nm)',fontsize=14)
ax1.legend(loc=6)


# fig1 = plt.figure()
# fig1.suptitle('Load at the elbow',fontsize=16)
# ax1 = fig1.add_subplot(111)
# l1 = ax1.plot(v1_elbow,label='Model I',linewidth=2.0)
# l2 = ax1.plot(v2_elbow,label='Model II',linewidth=2.0)
# ax1.set_ylim([0,8.5])
# ax1.set_xlabel('Time step')
# ax1.set_ylabel('Magnitude of moment (Nm)')
# ax1.legend(loc=2)
#
# fig2 = plt.figure()
# fig2.suptitle('Load at the shoulder',fontsize=16)
# ax2 = fig2.add_subplot(111)
# l3 = ax2.plot(v1_shoulder,label='Model I',linewidth=2.0)
# l4 = ax2.plot(v2_shoulder,label='Model II',linewidth=2.0)
# ax2.set_xlabel('Time step')
# ax2.set_ylabel('Magnitude of moment (Nm)')
# ax2.set_ylim([15,25])
# ax2.legend(loc=2)

plt.show()