import numpy as np
import math
import logging

class inverse_kinematics_solver:
    """ class that is a inverse kinematics solver for the third arm """

    def __init__(self):

        self.horizontal_pan_len = -.08
        self.wrist_len = .045
        self.end_effector_len = .135
        self.zero = 1e-8
        self.half_pi = math.pi/2


    def solve_kinematics(self, input_matrix):
        """ used to perform inverse kinematics on the wearable third robotic arm 

        returns True if successful matrix found, False if not matrix found
        also returns matrix itself    

        """

        output_joints = np.zeros([1,5])

        # check rank
        if np.linalg.matrix_rank(input_matrix) != 4:
            print('The Forward Kinematics Matrix is Singular. Cannot solve for joint angles.')
            return False, output_joints

        # flatten input matrix to be more easily used
        flat_IM = np.reshape(input_matrix, [16,1])

        # when R2z is > 0
        if abs(flat_IM[6]) > zero:

            output_joints[0] = thet1(flat_IM, self.end_effector_len)

            c4, output_joints[1] = thet2(output_joints[0], flat_IM)

            output_joints[3] = thet4(output_joints[1], c4, flat_IM)

            output_joints[4] = thet5(output_joints[1], c4, flat_IM)

        # when R2z == 0
        else:
            output_joints = self.zero_R2_calculation(output_joints)

        flat_IM[2] = thet3(output_joints, flat_IM)

        if constraint_check(flat_IM, output_joints):
            output_joints[2]= -1

        return self.check_constraints(output_joints)

    def check_contraints(self, output_joints):
        
        # FK from IK predictions:
        TIK = np.identity(4)

        alphas = [self.half_pi , self.half_pi , 0 , self.half_pi , self.half_pi, 0]

        ais = [0, 0, 0 , 0, 0, self.end_effector_len]
        
        di = [l1, 0, output_joints[2] , self.wrist_len, 0, 0]
        # TODO
        thetas = [output_joints, 0]
        thetas[2] = math.pi

        for i in range(len(thetas)):
            T_new = np.array(
                [cos(thetas[i]), -cos(alphas[i])*sin(thetas[i]),sin(alphas[i])*sin(thetas[i]), ais[i]*cos(thetas[i]),
                            sin(thetas[i]), cos(alphas[i])*cos(thetas[i]), -sin(alphas[i])*cos(thetas[i]), ais[i]*sin(thetas[i]),
                            0, sin(alphas[i]), cos(alphas[i]), di[i],
                            0, 0, 0, 1])
            TIK = TIK*T_new
        
        if np.linalg.matrix_rank(TIK) == 4:
            return True, output_joints
        else:
            msg = 'The reconstructed Forward Kinematics Matrix is Singular.'
            logging.log(5, msg)
            print(msg)
            return False, output_joints

    def zero_R2_calculation(self, output_joints):
        """ calculations for joints when R2 is 0 """
        output_joints[3] = 0
        R1x = flat_IM[0]
        R1y = flat_IM[1]
        R1z = flat_IM[2]
        px = flat_IM[12]
        py = flat_IM[13]
        pz = flat_IM[14]
        
        output_joints[0] = self.theta_1(flat_IM, self.end_effector_len)
        np.array([px - R1x * self.end_effector_len, py - R1y * self.end_effector_len, pz - R1z * self.end_effector_len])

        t1 = abs(self.horizontal_pan_len)

        tempArray =  self.np.array([px - R1x*self.end_effector_len, py - R1y*self.end_effector_len, pz - R1z*self.end_effector_len])

        t2 = np.linalg.norm(tempArray)

        tempArray[2] -= self.horizontal_pan_len

        t3 = np.linalg.norm(tempArray)

        thet2_cand = math.pi - math.acos((t1*t1 + t3*t3 - t2*t2) / (2 * t1 * t3))

        output_joints[1]= abs(thet2_cand)
        output_joints[4] = thet5(output_joints[1], 1, flat_IM)

        return output_joints

    # TODO rewrite to be readable
    def thet_1(flattened_input_matrix, var_three):
        #Range: [-pi,pi]
        r1y = flattened_input_matrix(1)
        r1x = flattened_input_matrix(0)
        py = flattened_input_matrix(13)
        px = flattened_input_matrix(12)

        return math.atan2(py - var_three*r1y, px - var_three*r1x)