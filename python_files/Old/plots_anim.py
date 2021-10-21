import numpy as np
import math
import pylab
from matplotlib import colors as mcolors
from matplotlib.collections import LineCollection
from matplotlib.collections import PolyCollection
import matplotlib.pyplot as plt
import csv
import matplotlib.animation as anim
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.tri as mtri

def read_data(fname):
    with open(fname, 'rb') as f:
        reader = csv.reader(f)
        # for row in reader:
        #     print row
        return list(reader)




def update(num,x1,y1,line1,x2,y2,line2):
    line1.set_data(x1[:num], y1[:num])
    line2.set_data(x2[:num], y2[:num])

    return line1, line2



shoulder_moments = read_data('v2_traj1_shoulder_moment.csv')
elbow_moments = read_data('v2_traj1_elbow_moment.csv')

fig1, ax = plt.subplots()
fig1.suptitle('Bio-Mechanical Loads for Fetching',fontsize=20)

y1 = shoulder_moments
x1 = list(range(0,len(shoulder_moments)))

y2 = elbow_moments
x2 = list(range(0,len(elbow_moments)))


line1, = ax.plot(x1, y1, color='r', label='Shoulder',linewidth=3.0)
line2, = ax.plot(x2, y2, color='b', label='Elbow',linewidth=3.0)

ax.set_ylim([0, 28])
ax.set_xlabel('Time step', fontsize=18)
ax.set_ylabel('Magnitude of moment (Nm)', fontsize=18)
ax.legend(loc=3, prop={'size': 18})


line_ani = anim.FuncAnimation(fig1, update, len(x1), fargs=[x1, y1, line1, x2, y2, line2], interval=20, blit=True)

line_ani.save('load1.mp4')

#fig2.suptitle('Bio-Mechanical Loads for Fetching',fontsize=20)
#ax2 = fig2.add_subplot(111)
#l1 = ax2.plot(shoulder_moments,label='Shoulder',linewidth=3.0, c='r')
#l2 = ax2.plot(elbow_moments,label='Elbow',linewidth=3.0, c='b',ls='dashed')
#ax2.set_ylim([0,28])
#ax2.set_xlabel('Time step',fontsize=18)
#ax2.set_ylabel('Magnitude of moment (Nm)',fontsize=18)
#ax2.legend(loc=3,prop={'size':18})

# shoulder_moments = read_data('v2_traj2_shoulder_moment.csv')
# elbow_moments = read_data('v2_traj2_elbow_moment.csv')
# fig3 = plt.figure()
# fig3.suptitle('Bio-Mechanical Loads for Assisted Handover',fontsize=20)
# ax3 = fig3.add_subplot(111)
# l1 = ax3.plot(shoulder_moments,label='Shoulder',linewidth=3.0, c='r')
# l2 = ax3.plot(elbow_moments,label='Elbow',linewidth=3.0, c='b',ls='dashed')
# ax3.set_ylim([0,28])
# ax3.set_xlabel('Time step',fontsize=18)
# ax3.set_ylabel('Magnitude of moment (Nm)',fontsize=18)
# ax3.legend(loc=3,prop={'size':18})

#plt.show()

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

#plt.show()