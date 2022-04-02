import ulab as np
from ulab import linalg
from ulab import vector as vc


class Kinematics(object):

    def __init__(self, delta_t):

        self.pi = 3.14159265
        self.delta_t = delta_t

        # H matrix at reference configuration 
        self.He0_atzero = np.array([[1, 0, 0.35], [0, 1, 0.25], [0, 0, 1]])

        return


    def find_qnew(self, m1_counts, m2_counts, pdot_desx, pdot_desy):
        """
        =INPUT=
            m1_counts - int
                The encoder counts of motor 1
            m2_counts - int
                The encoder counts of motor 2
            pdot_desx - float
                The desired velocity in x direction
            pdot_desy - float
                The desired velocity in y direction
        =OUTPUT=
            m1_counts_new - int
                The new encoder counts 
            m2_counts_new - int
                The new encoder counts 

        Calls all the functions below and apply them in the correct orden
        """
        # Set the desired velocity 
        self.pdot_des = np.array([[pdot_desx], [pdot_desy]])

        # Transform from counts to radians and from motor angles to joint angles
        m1, m2 = self.from_counts_to_radians(m1_counts, m2_counts)
        q1, q2 = self.from_motor_to_joints(m1, m2)

        # Set the vector of the curent angles of the joints
        q = np.array([[q1], [q2]])

        # Inverse kinematics
        Ad_H0e = self.get_Ad_H0e(q1, q2)
        J = self.get_jacobian(q1)
        q1_new, q2_new = self.inverse_kinematics(Ad_H0e, J, q)
        
        # Transform from radians to counts and from joint angles to motor angles
        m1_new, m2_new = self.from_joints_to_motor(q1_new, q2_new)
        m1_counts_new, m2_counts_new = self.from_radians_to_counts(m1_new, m2_new)

        # Round the values for a better understanding of them
        m1_counts_new = round(m1_counts_new)
        m2_counts_new = round(m2_counts_new)

        return m1_counts_new, m2_counts_new

    def get_Ad_H0e(self, q1, q2):
        """
        =INPUT=
            q1 - float
                The angle of the joint number 1
            q2 - float
                The angle of the joint number 2

        Calculate the adjoint matrix of the H matrix from 0 to the end-effector
        """
        # Find the exponential Twists for reference configuration
        e_T1_atzero = np.array([[vc.cos(q1), -vc.sin(q1), 0],
                                [vc.sin(q1), vc.cos(q1), 0],
                                [0, 0, 1]])

        e_T2_atzero = np.array([[vc.cos(q2), -vc.sin(q2), 0.25*vc.sin(q2)],
                                [vc.sin(q2), vc.cos(q2), 0.25 - 0.25*vc.cos(q2)],
                                [0, 0, 1]])

        # Find H matrix from end-effector to 0
        e_tot = linalg.dot(e_T1_atzero, e_T2_atzero)
        He0 = linalg.dot(e_tot, self.He0_atzero)

        # Find the inverse of He0 -> H0e
        H0e = np.array([[He0[0][0], He0[1][0], -(He0[0][0]*He0[0][2]+He0[1][0]*He0[1][2])], 
                        [He0[0][1], He0[1][1], -(He0[0][1]*He0[0][2]+He0[1][1]*He0[1][2])],
                        [0, 0, 1]])

        # Find the Adjoint matrix of H0e
        Ad_H0e = np.array([[1, 0, 0], [H0e[1][2], H0e[0][0], H0e[0][1]], [-H0e[0][2], H0e[1][0], H0e[1][1]]])

        return Ad_H0e

    def get_jacobian(self, q1):
        """
        Give the jacobian matrix which depends on the angle of q1
        """
        J = np.array([[1, 1],
                      [0, 0.25*vc.cos(q1)],
                      [0, 0.25*vc.sin(q1)]])
        return J

    def inverse_kinematics(self, Ad_H0e, J, q):
        """
        =INPUT=
            Ad_H0e - matrix
                Adjoint matrix
            J - matrix
                Jacobian matrix
            q - vector
                the current joint angles 
        =OUTPUT=
            q1_new - float
                The new angle of joint 1 
            q2_new - float
                The new angle of joint 2
                
        Makes use of modified inverse kinematics wich will be given a velocity desired and will output a position
        """
        # Multiply Ad_H0e with J
        J_modified = linalg.dot(Ad_H0e, J)

        # Remove the angular velocity from J_modified
        J_mod_2x2 = J_modified[1:3][:]

        # Find inverse of J_mod_2x2
        J_mod_inv = linalg.inv(J_mod_2x2)

        # Multiply final Jacobian with the desired velocity profile
        q_dot = linalg.dot(J_mod_inv, self.pdot_des)

        # Find new qs
        q_new = q + q_dot*self.delta_t
        q1_new = q_new[0][0]
        q2_new = q_new[1][0]

        return q1_new, q2_new

    def from_counts_to_radians(self, m1_counts, m2_counts):
        """
        One rotation of the main shaft is 50400 counts, this will be transformed into radias
        """
        m1 = m1_counts * 2*self.pi / 50400
        m2 = m2_counts * 2*self.pi / 50400
        return m1, m2


    def from_motor_to_joints(self, m1, m2):
        """
        m1 is the count of the motor that rotate the first section of the arm
        m2 is the count of the motor that rotates the second section of the arm
        """
        q1 = m1
        q2 = m2 
        return q1, q2


    def from_radians_to_counts(self, m1_new, m2_new):
        """
        
        """
        m1_counts_new = m1_new * 50400 / (2*self.pi)
        m2_counts_new = m2_new * 50400 / (2*self.pi)
        return m1_counts_new, m2_counts_new


    def from_joints_to_motor(self, q1_new, q2_new):
        """
        
        """
        m1_new = q1_new
        m2_new = q2_new
        return m1_new, m2_new
