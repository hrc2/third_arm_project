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

def write_data(fname,datavec):
    with open(fname, 'a') as f:
        writer = csv.writer(f)
        for data in datavec:
            writer.writerow([data])






# motor_torques = np.array(read_data('demo_task_1.csv'))
# start = 1600
# end = 40000
# motor1 = np.absolute(motor_torques[start:end,0].astype(float))
# motor2 = np.absolute(motor_torques[start:end,1].astype(float))
# motor3 = motor_torques[start:end,2]
#
# Nx = end-start-2000
# x = np.linspace(0,(end-2000-start)/10000.0,num=Nx)
# m1plot = motor1[0:len(x)]
# m2plot = motor2[0:len(x)]

#write_data("for_smoothing_fetch_dof_1.csv",m1plot)
#write_data("for_smoothing_fetch_dof_2.csv",m2plot)

m1plot = np.absolute(np.array(read_data('motor1_fetch_filtered.csv')).astype(float))
m2plot = np.absolute(np.array(read_data('motor2_fetch_filtered.csv')).astype(float))
x = np.linspace(0,len(m1plot)/10000.0,num=len(m1plot))

fig1 = plt.figure()
fig1.suptitle('Torque Load on Motors while Fetching',fontsize=20)
ax1 = fig1.add_subplot(111)
l1 = ax1.plot(x, m1plot ,label='DoF 1',linewidth=2.0,c='r')
l2 = ax1.plot(x, m2plot ,label='DoF 2',linewidth=2.0,c='g')
#l3 = ax1.plot(motor3,label='DoF 3 : Length Extension',linewidth=2.0,c='b')
#l2 = ax1.plot(v2_elbow,label='Model II',linewidth=2.0)
ax1.autoscale()
ax1.set_ylim([0,0.53])
ax1.set_xlim([x[0],x[-1]])
ax1.set_xlabel('Time (s)',fontsize=18)
ax1.set_ylabel('Ratio of Torque to Peak Motor Torque',fontsize=18)
ax1.legend(loc=2,prop={'size':18})
#ax1.set_xticks([])
plt.show()




