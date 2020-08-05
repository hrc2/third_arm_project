#!/usr/bin/env python
import rospy

import math
import time
import numpy as np
import scipy.signal as sg

def rotm_from_vecs(vec1, vec2):
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    if np.linalg.norm(v) < 10 ** -10:
        rotation_matrix = np.eye(3)
    else:
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

def ik_3d_pos(pos):
    l1 = -0.08
    l2 = 0.10
    
    x = pos[0]
    y = pos[1]
    z = pos[2]

    thet1 = np.arctan2(y,x);
    thet3 = np.sqrt(x**2 + y**2 + (z-l1)**2) - l2;
    thet2 = np.arccos(np.abs(z-l1)/(l2+thet3))

    return [thet1, thet2, thet3]

def mocap_filter(data):
    # Transfer function for IIR low-pass filters, cutoff at 12 Hz
    b = np.array([0.1400982208, -0.0343775491, 0.0454003083, 0.0099732061, 0.0008485135])
    a = np.array([1, -1.9185418203, 1.5929378702, -0.5939699187, 0.0814687111])

    data_filt = sg.filtfilt(b, a, x=data, axis=0)
    diff = np.diff(data, axis=0)
    mean_dx = np.mean(diff, axis=0)
    alpha = 0.02

    return alpha*mean_dx + (1-alpha)*data_filt, mean_dx
