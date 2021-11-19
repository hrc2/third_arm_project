from scipy.spatial.transform import Rotation as R
import numpy as np
from inverse_kinematics import InverseKinematicsSolver

def get_transform_from_translation_and_rotation(translation, rotation):
    """ get the transformation matrix from the tranlsation and rotation arrays """

    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_quat.html
    # http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
    # https://stackoverflow.com/questions/40833073/insert-matrix-into-the-center-of-another-matrix-in-python

    rotation_matrix = R.from_quat(rotation)
    rotation_matrix = rotation_matrix.as_matrix()
    # assemble transform

    transform = np.zeros((4, 4))

    # add rotation
    transform[0:3, 0:3] = rotation_matrix
    
    # add transform
    translation = np.array(translation)
    transform[0:3, 3:4] = translation.reshape((3, 1))

    transform[3][3] = 1

    return transform

t1 = [4, 5, 6]

r1 = [1, 2, 3, 4]

t2 = [8, 9, 10]

r2 = [7, 8, 3, 4]

a = (get_transform_from_translation_and_rotation(t1, r1))
b = (get_transform_from_translation_and_rotation(t2, r2))
c = np.matmul(np.linalg.inv(a), b)

solver = InverseKinematicsSolver()

solver.solve_kinematics(c)