#!/usr/bin/env python
import rospy

import math
import time
import numpy as np

def rotm_from_vecs(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
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

