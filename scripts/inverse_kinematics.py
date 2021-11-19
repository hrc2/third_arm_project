import numpy as np
import math
import logging


class InverseKinematicsSolver:
    """ class that is a inverse kinematics solver for the third arm """

    def __init__(self):

        self.horizontal_pan_len = -.08
        self.wrist_len = .045
        self.end_effector_len = .135
        self.zero = 1e-8
        self.half_pi = math.pi / 2

    def solve_kinematics(self, input_matrix):
        """ used to perform inverse kinematics on the wearable third robotic arm

        returns success, np.matrix
        success is True if successful matrix found, False if not matrix found

        np.matrix is the output thetas
        """
        output_joints = np.zeros(5)

        # check rank
        if np.linalg.matrix_rank(input_matrix) != 4:
            print('The Forward Kinematics Matrix is Singular. Cannot solve for joint angles.')
            return False, output_joints

        # flatten input matrix to be more easily used
        flat_IM = np.reshape(input_matrix, (16, 1))

        # when R2z is > 0
        if abs(flat_IM[6]) > self.zero:

            output_joints[0] = self.theta_1(flat_IM)

            c4, output_joints[1] = self.theta_2(output_joints[0], flat_IM)

            output_joints[3] = self.theta_4(c4, flat_IM)

            output_joints[4] = self.theta_5(c4, flat_IM)
            # TODO test output_joints here with a print to make sure no arrays got messed up
        
        # when R2z == 0
        else:
            output_joints = self.zero_R2_calculation(output_joints)

        output_joints[2] = self.theta_3(output_joints, flat_IM)

        if not self.is_constraint_check_valid(flat_IM, output_joints):
            return False, output_joints

        return self.check_IK_constraints(output_joints)

    def check_IK_constraints(self, output_joints):

        # FK from IK predictions:
        TIK = np.identity(4)

        alphas = [self.half_pi, self.half_pi, 0, self.half_pi, self.half_pi, 0]

        ais = [0, 0, 0, 0, 0, self.end_effector_len]

        di = [self.horizontal_pan_len, 0, output_joints[2], self.wrist_len, 0, 0]
        thetas = output_joints.tolist()
        thetas.append(0)
        thetas[2] = math.pi

        for i in range(len(thetas)):
            T_new = np.array(
                [[math.cos(thetas[i]), -math.cos(alphas[i]) * math.sin(thetas[i]), math.sin(alphas[i]) * math.sin(thetas[i]), ais[i] * math.cos(thetas[i])],
                 [math.sin(thetas[i]), math.cos(alphas[i]) * math.cos(thetas[i]), -math.sin(alphas[i]) * math.cos(thetas[i]), ais[i] * math.sin(thetas[i])],
                 [0, math.sin(alphas[i]), math.cos(alphas[i]), di[i]],
                 [0, 0, 0, 1]])
            TIK = np.matmul(TIK, T_new)

        if np.linalg.matrix_rank(TIK) == 4:
            return True, output_joints
        else:
            msg = 'The reconstructed Forward Kinematics Matrix is Singular.'
            logging.log(5, msg)
            print(msg)
            return False, output_joints

    def zero_R2_calculation(self, output_joints, flat_IM):
        """ calculations for joints when R2 is 0 """
        output_joints[3] = 0
        R1x = flat_IM[0]
        R1y = flat_IM[1]
        R1z = flat_IM[2]
        px = flat_IM[12]
        py = flat_IM[13]
        pz = flat_IM[14]

        output_joints[0] = self.theta_1(flat_IM)

        t1 = abs(self.horizontal_pan_len)

        tempArray = self.np.array(
            [px - R1x * self.end_effector_len, py - R1y * self.end_effector_len, pz - R1z * self.end_effector_len])

        t2 = np.linalg.norm(tempArray)

        tempArray[2] -= self.horizontal_pan_len

        t3 = np.linalg.norm(tempArray)

        output_joints[1] = abs(math.pi - math.acos((t1 * t1 + t3 * t3 - t2 * t2) / (2 * t1 * t3)))
        
        output_joints[4] = self.theta_5(1, flat_IM)

        return output_joints

    def theta_1(self, flat_IM):
        # Range: [-pi,pi]
        r1y = flat_IM[1]
        r1x = flat_IM[0]
        py = flat_IM[13]
        px = flat_IM[12]

        return math.atan2(py - self.end_effector_len * r1y, px - self.end_effector_len * r1x)

    def theta_2(self, t1, flat_TM):
        # Range: [0, pi / 2]

        r2x = flat_TM[4].item()
        r2y = flat_TM[5].item()
        r2z = flat_TM[6].item()

        A = np.asarray([[math.sin(t1), math.cos(t1) * r2z], [-math.cos(t1), math.sin(t1) * r2z]])

        numerator = [[r2x] ,[r2y]]

        avec = np.linalg.lstsq(A, numerator, rcond=-1)[0]

        #cos(t4) is also found
        c4 = avec[0][0] # TODO which want? [10] or [1]? probably 0 since matches other

        cand = math.atan2(1, avec[1])

        if cand > 0 and cand < self.half_pi:
            t2 = cand
        elif cand > self.half_pi and cand <= math.pi:
            t2 = -cand + math.pi

        elif cand < -self.half_pi and cand > -math.pi:
            t2 = math.pi + cand
        else:
            t2 = -cand

        return c4, t2

    def theta_3(self, output_joints, flat_IM):

        #Range: [0.33,0.45]

        st = 0.33 #Start
        en = 0.45 #End

        t1 = output_joints[0]
        t2 = output_joints[1]
        t4 = output_joints[3]
        t5 = output_joints[4]

        thresh = 1e-8
        pz = flat_IM[14]
        px = flat_IM[12]
        py = flat_IM[13]
        l1 = self.horizontal_pan_len
        l2 = self.wrist_len
        l3 = self.end_effector_len

        d3z_Nan = math.cos(t2)
        d3y_Nan = math.sin(t1)*math.sin(t2)
        d3x_Nan = math.cos(t1)*math.sin(t2)

        d3z = 0
        d3y = 0
        d3x = 0

        if d3z_Nan != 0:
            d3z = -pz/d3z_Nan + l1/d3z_Nan - l3*math.sin(t5) - l3*math.cos(t4)*math.cos(t5)*math.tan(t2) - l2

        if d3y_Nan != 0:
            d3y = py / d3y_Nan - l2 - l3 * math.sin(t5) - l3 * (math.cos(t5) * math.cos(t1) * math.sin(t4) - math.cos(t5) * math.sin(t1) * math.cos(t2) * math.cos(t4)) / d3y_Nan

        if d3x_Nan != 0:
            d3x = px/d3x_Nan - l2 - l3*math.sin(t5) + l3*(math.cos(t5)*math.sin(t1)*math.sin(t4) + math.cos(t5)*math.cos(t1)*math.cos(t2)*math.cos(t4))/d3x_Nan


        f1 = 0
        f2 = 0
        f3 = 0

        if d3x <= en and d3x >= st:
            f1 = 1
        if d3y <= en and d3y >= st:
            f2 = 1
        if d3z <= en and d3z >= st:
            f3 = 1

        f_check = (abs(math.sin(t1)) < thresh or abs(math.sin(t2)) < thresh)
        f_check_2 = (d3z == 0 or abs(math.cos(t2)) < thresh)

        if f1+f2+f3 == 3:
            d3 = (d3x + d3y + d3z)/3.0
        elif (f1+f2 == 2) and (d3z == 0 or abs(math.cos(t2)) < thresh):
             d3 = (d3x+d3y)/2.0
        elif (f1+f3 == 2) and (d3y == 0 or f_check):
             d3 = (d3x+d3z)/2.0
        elif (f2+f3 == 2) and (d3x == 0 or abs(math.cos(t1)) < thresh or abs(math.sin(t2)) < thresh):
             d3 = (d3y+d3y)/2.0
        elif f1 and (d3y == 0 or f_check) and f_check_2:
             d3 = d3x
        elif f2 and (d3x == 0 or f_check) and f_check_2:
            d3 = d3y
        elif f3 and (f_check or (d3x == 0 and d3y == 0)):
            d3 = d3z
        else:
            d3 = -1 # No solution found

        return d3

    def theta_4(self, c4, flat_IM):
        # Range: [-pi, pi]
        r2z = flat_IM[6]
        s4 = -r2z / math.sin(self.wrist_len)

        return math.atan2(s4, c4)

    def theta_5(self, c4, flat_IM):
        """
        Range: [0,pi]
        """
        c4 = c4.item()
        r1z = flat_IM[2].item()
        r3z = flat_IM[10].item()

        A = np.array([[-math.cos(self.wrist_len), -c4*math.sin(self.wrist_len)], [-c4*math.sin(self.wrist_len), math.cos(self.wrist_len)]])

        numerator = [[r1z], [r3z]]
        cvec = np.linalg.lstsq(A, numerator, rcond=-1)[0]

        return math.atan2(cvec[0], cvec[1])

    def is_constraint_check_valid(self, flat_IM, output_joints):
        """ check contraints of output_joints """
        
        r1z = flat_IM[2].item()
        r2z = flat_IM[6].item()
        r3z = flat_IM[10].item()
        r2x = flat_IM[4].item()
        r2y = flat_IM[5].item()
        thresh = 1e-10
        
        # First constraint equation, c1 should be ~=0
        c1 = r1z*math.cos(output_joints[4])*math.sin(output_joints[3]) - r2z*math.cos(output_joints[3]) + r3z*math.sin(output_joints[3])*math.sin(output_joints[4])
        if abs(c1) > thresh:
            return False
        
        # Second constraint equation, c2 should be ~=0
        c2 = r2x*cos(output_joints[0])*sin(output_joints[1]) - r2z*cos(output_joints[1]) + r2y*sin(output_joints[0])*sin(output_joints[1])
        if abs(c2) > thresh:
            return False

        return True