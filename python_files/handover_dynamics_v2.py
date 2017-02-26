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

def find_force(d3,thet,phi): #Force, moment at shoulder and elbow
    i = np.array([1,0,0])
    j = np.array([0,1,0])
    k = np.array([0,0,1])

    l1 = 0.366
    l2 = 1

    m1 = 2.5
    m2 = 1.45
    m3 = 0.6
    m4 = 0.3
    m5 = 0.6
    g = 9.81

    #d2 = 1 #Variable
    #thet = 1 #Variable
    #phi = 1#Variable
    a = 0.075
    d = 0.016
    d1 = 0.157/2
    d2 = 0.264/2 - 0.157/2


    ang = math.radians(-25)

    rg1 = (l1/2)*(math.cos(ang)*i + math.sin(ang)*k)
    rb = 2*rg1

    rg2 = rb + (l2/2)*i
    rd = rb + d*i + a*(-k)

    d1hat = (math.cos(thet)*i + math.sin(thet)*j)
    d2hat = (math.cos(phi)*d1hat + math.sin(phi)*k)
    rg3 = rd + d1*d1hat
    rg4 = rg3 + d2*d2hat
    rg5 = rg4 + d3*d2hat

    #Shoulder
    ms = np.array([m1,m2,m3,m4,m5])
    rs = [rg1,rg2,rg3,rg4,rg5]

    fa = g*np.sum(ms)*k
    ma = 0
    for t in range(np.size(ms)):
        ma += ms[t]*g*np.cross(rs[t],k)

    #Elbow
    me = np.array([m2,m3,m4,m5])
    re = [rg2,rg3,rg4,rg5] - rb

    fb = g*np.sum(me)*k
    mb = 0
    for t in range(np.size(me)):
        mb += me[t]*g*np.cross(re[t],k)

    return fa,ma,fb,mb





def moment_v2_traj1():
    fs_mag = []
    fe_mag = []
    ms_mag = []
    me_mag = []

    fs = []
    ms = []
    fe = []
    me = []

    N = 100
    thets = np.linspace(0, 1.00, num=N)
    d3 = np.linspace(0.256-0.157/2, 0.256+0.160-0.157/2, num=N)
    phi = np.linspace(0, 0.1, num=N)

    for i in range(N):
        f1,m1,f2,m2 = find_force(d3[i],thets[i],phi[i])
        fs.append(f1)
        ms.append(m1)
        fe.append(f2)
        me.append(m2)
        fs_mag.append(np.linalg.norm(f1))
        fe_mag.append(np.linalg.norm(f2))
        ms_mag.append(np.linalg.norm(m1))
        me_mag.append(np.linalg.norm(m2))

    N = 200
    thets = np.linspace(thets[-1], -3.00, num=N)
    d3 = np.linspace(d3[-1], d3[0], num=N)
    phi = np.linspace(phi[-1], 0.3, num=N)

    for i in range(N):
        f1,m1,f2,m2 = find_force(d3[i],thets[i],phi[i])
        fs.append(f1)
        ms.append(m1)
        fe.append(f2)
        me.append(m2)
        fs_mag.append(np.linalg.norm(f1))
        fe_mag.append(np.linalg.norm(f2))
        ms_mag.append(np.linalg.norm(m1))
        me_mag.append(np.linalg.norm(m2))

#    fs = np.vstack(moments)
#     plt.plot(ms_mag, c='r')
#     plt.plot(me_mag, c='b')
#     plt.show()
    return ms_mag,me_mag,fs_mag,fe_mag




shoulder_moments, elbow_moments,sf,ef = moment_v2_traj1()

# with open('fetching_shooulder.csv','wb') as v2s:
#     writer = csv.writer(v2s)
#     for val in shoulder_moments:
#         writer.writerow([val])
#
# with open('fetching_elbow.csv','wb') as v2e:
#     writer = csv.writer(v2e)
#     for el in elbow_moments:
#         writer.writerow([el])

fig1 = plt.figure()
fig1.suptitle('Bio-Mechanical Loads for Assisted Handover',fontsize=20)
ax1 = fig1.add_subplot(111)
l1 = ax1.plot(shoulder_moments,label='Shoulder',linewidth=3.0, c='r')
l2 = ax1.plot(elbow_moments,label='Elbow',linewidth=3.0, c='b',ls='dashed')
ax1.set_ylim([0,24])
ax1.set_xlabel('Time step',fontsize=14)
ax1.set_ylabel('Magnitude of moment (Nm)',fontsize=14)
ax1.legend(loc=2)

# fig2 = plt.figure()
# fig2.suptitle('Force loads for Assisted Handover',fontsize=20)
# ax1 = fig2.add_subplot(111)
# l1 = ax1.plot(sf,label='Shoulder',linewidth=3.0, c='r')
# l2 = ax1.plot(ef,label='Elbow',linewidth=3.0, c='b',ls='dashed')
# ax1.set_ylim([0,60])
# ax1.set_xlabel('Time step')
# ax1.set_ylabel('Magnitude of force (N)')
# ax1.legend(loc=1)

plt.show()