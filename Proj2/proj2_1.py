#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion, quaternion_matrix

from math import atan2, sqrt, pi, acos, sin, cos, asin
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from scipy.linalg import logm, expm
import numpy as np

class ourAPI:
    def __init__(self):
        self.H1 = 89.45
        self.H2 = 100
        self.L1 = 35
        self.L2 = 100
        self.L3 = 107.6
        self.S = np.transpose(np.array([[0, 0, 1, 0, 0, 0],
                            [0, 1, 0, -self.H1, 0, 0],
                            [0, 1, 0, -(self.H1+self.H2), 0, self.L1],
                            [0, 1, 0, -(self.H1+self.H2), 0, self.L1+self.L2]]) )
        self.M = np.array([[1,0,0,self.L1+self.L2+self.L3],[0,1,0,0],[0,0,1,self.H1+self.H2],[0,0,0,1]])
        self.a = [[0,0,0],[0,0,self.H1],[self.L1,0,self.H1+self.H2],[self.L1+self.L2,0,self.H1+self.H2]]
        
        self.Tsd = np.eye((4))

    def skew_symmetry(self, vec):
        skew_matrix = np.array([[ 0, -vec[2], vec[1]],
                                [vec[2], 0, -vec[0]],
                                [-vec[1], vec[0], 0]])
        return skew_matrix

    def screw_axis_to_transformation_matrix(self, screw_axis, angle):
        # screw_axis should be np.array([sw1 sw2, sw3, sv1, sv2, sv3])
        assert len(screw_axis) == 6, "Input screw axis must have six components"

        # Extract rotational and translational components from the screw axis
        Sw = screw_axis[:3]
        Sv = screw_axis[3:]

        # Matrix form of the screw axis
        screw_matrix = np.zeros((4, 4))
        screw_matrix[:3, :3] = self.skew_symmetry(Sw)
        screw_matrix[:3, 3] = Sv

        # Exponential map to get the transformation matrix
        exponential_map = expm(angle * screw_matrix)
        
        return exponential_map
    
    def twist_vector_from_twist_matrix(self, twist_matrix):
        """
        Compute the original 6D twist vector from a 4x4 twist matrix.

        Parameters:
        - twist_matrix: A 4x4 matrix representing the matrix form of the twist 

        Returns:
        - twist_vector: The 6D twist vector [w, v] corresponding to the input
                        twist matrix.
        """
        assert twist_matrix.shape == (4, 4), "Input matrix must be 4x4"

        w = np.array([twist_matrix[2, 1], twist_matrix[0, 2], twist_matrix[1, 0]])
        v = twist_matrix[:3, 3]

        return np.concatenate((w, v))
    
    def body_jacobian(self, JS, Tb_s):
        R = Tb_s[:3, :3]
        r = Tb_s[:3, 3]
        p = self.skew_symmetry(r)
        pR= p @ R
        ad_tbs = np.block([[R, np.zeros((3, 3))],
                        [pR, R]])
        JB = ad_tbs @ JS
        
        return JB
    
    def Space_Jacobian(self, angles):
        a = self.a
        q = angles
        n = len(q)
        JS = np.zeros((6, n))
        T = np.eye(4,4)
        Ts_b = np.eye(4,4)
        rot = np.array([self.S[:3, 0], self.S[:3, 1], self.S[:3, 2], self.S[:3, 3]])
        ad = {}
        tf = {}
          
        for ii in range(n-1,-1,-1):
            rot_hat = self.skew_symmetry(rot[ii])
            e_rot_hat=np.eye(3,3)+rot_hat*np.sin(q[ii])+rot_hat@rot_hat*(1-np.cos(q[ii]))

            if ii>0:
                Sv = -np.transpose(np.cross(rot[ii][:], a[ii][:]))
            elif ii==0:
                Sv = [0, 0, 0]
            
            p = ((np.eye((3))*q[ii]+(1-np.cos(q[ii]))*rot_hat+(q[ii]-np.sin(q[ii]))*rot_hat@rot_hat)@Sv).reshape(3,1)
            e_zai = np.block([[e_rot_hat, p], [0, 0, 0, 1]])
            T = e_zai@T

            tf[ii+1] = e_zai
    
        Ts_b = T @ self.M 
        Tb_s = np.linalg.inv(Ts_b)

        for num in range(n):
            tf_cumu = np.eye(4)
            for num_tf in range(num+1):
                tf_cumu = tf_cumu @ tf[num_tf+1]
            R = tf_cumu[:3, :3]
            p = tf_cumu[:3, 3]
            p_hat = self.skew_symmetry(p)

            ad[num] = np.block([
                [R,              np.zeros((3, 3))],
                [p_hat @ R,      R]
            ])
            
        for num_Js in range(n):
            if num_Js == 0:
               JS[:, [num_Js]] = self.S[:, [num_Js]]
            else:
               JS[:, [num_Js]] = ad[num_Js] @ self.S[:, [num_Js]]
                
        return JS, Ts_b, Tb_s

    def num_IK(self, Tsd, InitGuess):
        for i in range(100):
            # Calculate the end-effector transform (Tsb) evaluated at the InitGuess using the helper functions that you wrote at the beginning.
            JS, Ts_b, Tb_s = self.Space_Jacobian(InitGuess) # s is base, b is end-effector
            Tbd = np.linalg.inv(Ts_b) @ Tsd
            matrix_Vb = logm(Tbd)
            Vb = self.twist_vector_from_twist_matrix(matrix_Vb).reshape((6, 1))

            # Compute new angles
            JB = self.body_jacobian(JS, Tb_s)
            JB_pseudoinv = np.linalg.pinv(JB)

            NewGuess = InitGuess+(JB_pseudoinv@ Vb).T[0]
            print(f"Iteration number: {i} \n")
            
            # Check if you're done and update initial guess
            if(np.linalg.norm(abs(NewGuess-InitGuess)) <= 0.001):
                return [NewGuess[0], NewGuess[1], NewGuess[2], NewGuess[3]] 
            else:
                InitGuess = NewGuess
        print('Numerical solution failed!!')

def main():
    my_api = ourAPI()
    # Determine the desired end-effector transform
    Td_grasp = np.array([[0,  -1,  0,  0],
                    [1,  0,  0,  my_api.L1+my_api.L2+my_api.L3],
                    [0,  0,  1, my_api.H1+my_api.H2],
                    [0,  0,  0,  1]]) # Gripping location
    Td_release = np.array([[0,  1,  0,  0],
                    [-1,  0,  0,  -(my_api.L1+my_api.L2+my_api.L3)],
                    [0,  0,  1,  my_api.H1+my_api.H2],
                    [0,  0,  0,  1]]) # Throwing location

    # Create experiment objects (use robot API + our custom API)
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )

    # Start with home position
    bot.arm.go_to_home_pose()

    # toggle between the geometric method and the numerical method below
    # record the answers that you get in both cases. report your observations. 

    # Go to gripping position and grip
    # joint_positions = my_api.geom_IK(Td_grasp) # Geometric inverse kinematics
    joint_positions = my_api.num_IK(Td_grasp, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.grasp(2.0) # Grip
    
    # Go to throwing position and throw
    # joint_positions = my_api.geom_IK(Td_release) # Geometric inverse kinematics
    joint_positions = my_api.num_IK(Td_release, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.release(2.0) # Release
    
    # End mission
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    bot.shutdown()


if __name__ == '__main__':
    main()




