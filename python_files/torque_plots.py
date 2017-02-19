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




motor_torques = np.array(read_data('demo_task_1.csv'))
start = 1600
end = 40000
motor1 = motor_torques[start:end,0]
motor2 = motor_torques[start:end,1]
motor3 = motor_torques[start:end,2]

Nx = end-start-2000
x = np.linspace(0,(end-2000-start)/10000.0,num=Nx)

fig1 = plt.figure()
fig1.suptitle('Torque Load on Motors',fontsize=16)
ax1 = fig1.add_subplot(111)
l1 = ax1.plot(x, motor1[0:len(x)] ,label='DoF 1 : Sideways Swivel',linewidth=2.0,c='r')
l2 = ax1.plot(x, motor2[0:len(x)] ,label='DoF 2 : Vertical Pitching',linewidth=2.0,c='g')
#l3 = ax1.plot(motor3,label='DoF 3 : Length Extension',linewidth=2.0,c='b')
#l2 = ax1.plot(v2_elbow,label='Model II',linewidth=2.0)
ax1.autoscale()
ax1.set_ylim([min(motor1),0.53])
ax1.set_xlim([x[0],x[-1]])
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Ratio of Torque to Peak Motor Torque')
ax1.legend(loc=2)
#ax1.set_xticks([])
plt.show()