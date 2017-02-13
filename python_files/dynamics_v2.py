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

def find_force(d2,thet):
    i = np.array([1,0,0])
    j = np.array([0,1,0])
    k = np.array([0,0,1])

    l1 = 0.366
    l2 = 1

    m1 = 2.5
    m2 = 1.45
    m3 = 0.9
    m4 = 0.6
    g = 9.81

    #d2 = 1 #Variable
    #thet = 1 #Variable
    a = 0.075
    d = 0.016
    d1 = 0.264/2

    ang = math.radians(-25)

    rg1 = (l1/2)*(math.cos(ang)*i + math.sin(ang)*k)
    rb = 2*rg1

    rg2 = rb + (l2/2)*i
    rd = rb + d*i + a*(-k)

    rg3 = rd + d1*(math.cos(thet)*i + math.sin(thet)*j)
    rg4 = rg3 + d2*(math.cos(thet)*i + math.sin(thet)*j)

    m = np.array([m1,m2,m3,m4])
    r = [rg1,rg2,rg3,rg4]

    fa = g*np.sum(m)*k
    ma = 0
    for t in range(np.size(m)):
        ma += m[t]*g*np.cross(r[t],k)

    return fa,ma


def find_force_e(d2,thet): #For the elbows
    i = np.array([1,0,0])
    j = np.array([0,1,0])
    k = np.array([0,0,1])

    l1 = 0.366
    l2 = 1

    m1 = 2.5
    m2 = 1.45
    m3 = 0.9
    m4 = 0.6
    g = 9.81

    #d2 = 1 #Variable
    #thet = 1 #Variable
    a = 0.075
    d = 0.016
    d1 = 0.264/2

    ang = math.radians(-25)

    rg1 = (l1/2)*(math.cos(ang)*i + math.sin(ang)*k)
    rb = 2*rg1

    rg2 = rb + (l2/2)*i
    rd = rb + d*i + a*(-k)

    rg3 = rd + d1*(math.cos(thet)*i + math.sin(thet)*j)
    rg4 = rg3 + d2*(math.cos(thet)*i + math.sin(thet)*j)

    m = np.array([m2,m3,m4])
    r = [rg2,rg3,rg4] - rb

    fa = g*np.sum(m)*k
    ma = 0
    for t in range(np.size(m)):
        ma += m[t]*g*np.cross(r[t],k)

    return fa,ma


def force_shoulders_v2():
    forces_mag = []
    moments_mag = []

    forces = []
    moments = []


    thets = np.linspace(math.radians(0),math.radians(-60),num=100)
    d2 = np.linspace(0.256,0.256+0.16,num=100)

    for thet in thets:
        force,moment = find_force(d2[0],thet)
        forces.append(force)
        moments.append(moment)
        forces_mag.append(np.linalg.norm(force))
        moments_mag.append(np.linalg.norm(moment))

    for dist in d2:
        force,moment = find_force(dist,thets[-1])
        forces.append(force)
        moments.append(moment)
        forces_mag.append(np.linalg.norm(force))
        moments_mag.append(np.linalg.norm(moment))

    thets = np.linspace(math.radians(-60),math.radians(60),num=100)

    for thet in thets:
        force,moment = find_force(d2[-1],thet)
        forces.append(force)
        moments.append(moment)
        forces_mag.append(np.linalg.norm(force))
        moments_mag.append(np.linalg.norm(moment))

#    fs = np.vstack(moments)
    plt.plot(moments_mag)
    plt.show()
    return moments_mag

def force_elbow_v2():
    forces_mag = []
    moments_mag = []

    forces = []
    moments = []


    thets = np.linspace(math.radians(0),math.radians(-60),num=100)
    d2 = np.linspace(0.256,0.256+0.16,num=100)

    for thet in thets:
        force,moment = find_force_e(d2[0],thet)
        forces.append(force)
        moments.append(moment)
        forces_mag.append(np.linalg.norm(force))
        moments_mag.append(np.linalg.norm(moment))

    for dist in d2:
        force,moment = find_force_e(dist,thets[-1])
        forces.append(force)
        moments.append(moment)
        forces_mag.append(np.linalg.norm(force))
        moments_mag.append(np.linalg.norm(moment))

    thets = np.linspace(math.radians(-60),math.radians(60),num=100)

    for thet in thets:
        force,moment = find_force_e(d2[-1],thet)
        forces.append(force)
        moments.append(moment)
        forces_mag.append(np.linalg.norm(force))
        moments_mag.append(np.linalg.norm(moment))

#    fs = np.vstack(moments)
    plt.plot(moments_mag)
    plt.show()
    return moments_mag


shoulder_moments = force_shoulders_v2()
elbow_moments = force_elbow_v2()

with open('v2_shoulder.csv','wb') as v2s:
    writer = csv.writer(v2s)
    for val in shoulder_moments:
        writer.writerow([val])

with open('v2_elbow.csv','wb') as v2e:
    writer = csv.writer(v2e)
    for el in elbow_moments:
        writer.writerow([el])