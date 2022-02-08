import math
import numpy as np
import os

def array_almost_equal(array_one, array_two, tolerance):
    """ determine if two arrays are almost equal

        Note we assume these could be lists so not done with numpy fastness
        
        Args:
            tolerance is a decimal
    
    """

    assert(len(array_one) == len(array_two))

    for index in range(0, len(array_one)):
        val_one = array_one[index]
        val_two = array_two[index]
        if val_one * val_two > 0:
            if abs(abs(val_one)-abs(val_two))/val_one < tolerance:
                pass
            else:
                return False
        else:
            return False

    return True


def generateFile(path, fileName, ending):
    # def generateFile(path: str, fileName: str, ending: str):
    """generates file inside path, ending is file ending like .txt, returns filePath"""

    filePath = pathCheck(fileName, path)

    # checking for duplicates
    count = 0  # so you can have same folder name but a different version
    while os.path.isfile(filePath + '_' + str(count) + ending):
        count += 1
    filePath = filePath + '_' + str(count) + ending
    file = open(filePath, 'w+')
    file.close()

    print('File for', fileName, 'created at path', filePath)

    return filePath


class PID_template:
    """ template class for a PID """

    def __init__(self, proportional, integral, derivative, minimum, maximum):
        """ instantiate PID"""
        self.integrator = 0
        self.derivator = 0
        self.previous = 0
        self.minimum = minimum
        self.maximum = maximum
        self.error = 0
        self.targetValue = 0

        # C means comes from config
        self.proportionalC = proportional
        self.integralC = integral
        self.derivativeC = derivative

    def update(self, calculated, processVariableCurrentValue):
        """ update PID """
        self.error = calculated - processVariableCurrentValue
        self.integrator += self.error
        self.derivator += self.previous - self.error
        self.previous = self.error
        self.targetValue = self.error * self.proportionalC + self.integralC * self.integrator + self.derivativeC * self.derivator

        if self.targetValue > self.maximum:
            self.targetValue = self.maximum

        elif self.targetValue < self.minimum:
            self.targetValue = self.minimum

        return self.targetValue


def getVectorAngle(vectorStart, vectorEnd, fromTop=False):
    """Get the vector angle from two points as lists, angle is from x = 1, y = 0"""
    vector = [vectorEnd[0] - vectorStart[0], vectorEnd[1] - vectorStart[1]]
    factor = np.linalg.norm(vector)
    if factor != 0:
        vector[0] /= factor
        vector[1] /= factor
    if fromTop:
        angle = math.acos((np.dot(vector, [0, 1])))
        angle *= getSide_2d([0, 1], vector) * -1
        if 3.14 < abs(angle) < 3.15:
            angle = 0
    else:
        angle = math.atan2(vector[1], vector[0])
        # it should be in this order: https://numpy.org/doc/stable/reference/generated/numpy.arctan2.html
        if angle < 0:
            angle = 6.28318 + angle
    return angle, vector


def getSide_2d(anchorVector, relativeVector):
    """get side of angle relativeVector relative to anchor vector, -1 for left, 1 for right"""
    # https://stackoverflow.com/questions/13221873/determining-if-one-2d-vector-is-to-the-right-or-left-of-another

    dot = anchorVector[0] * -relativeVector[1] + anchorVector[1] * relativeVector[0]

    if dot >= 0:
        return 1
    else:
        return -1

