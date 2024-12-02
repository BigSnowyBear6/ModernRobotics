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
        # super().__init__('inverse_kinematics_node')
        # self.subscription = self.create_subscription(
        #     Pose,
        #     'desired_pose',
        #     self.pose_callback,
        #     10)
        # self.publisher_ = self.create_publisher(JointState, 'joint_command', 10)
        # self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']  

        self.H1 = 89.45/1000
        self.H2 = 100/1000
        self.L1 = 35/1000
        self.L2 = 100/1000
        self.L3 = 107.6/1000
        self.S = np.transpose(np.array([[0, 0, 1, 0, 0, 0],
                            [0, 1, 0, -0.08945, 0, 0],
                            [0, 1, 0, -0.18945, 0, 0.035],
                            [0, 1, 0, -0.18945, 0, 0.135]]) )
        self.M = np.array([[1,0,0,self.L1+self.L2+self.L3],[0,1,0,0],[0,0,1,self.H1+self.H2],[0,0,0,1]])
        self.a = [[0,0,0],[0,0,self.H1],[self.L1,0,self.H1+self.H2],[self.L1+self.L2,0,self.H1+self.H2]]
        
        self.Tsd = np.eye((4))
        # self.angle_guess = np.zeros(6)
        self.angle_guess = np.zeros(4)

    # def pose_callback(self, msg):
    #    # Extract translation
    #    position = msg.position
    #    desired_p = np.array([[position.x], [position.y], [position.z]])
    #    self.get_logger().info(f"Received Desired Pose:")
    #    self.get_logger().info(f"  Position -> {desired_p}")

    #    # Extract orientation (quaternion) and convert to a rotation matrix
    #    orientation = msg.orientation
    #    desired_q = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
    #    desired_rot = quaternion_matrix(desired_q)[:3, :3]
    #    self.get_logger().info(f"  Orientation -> {desired_rot}")
       
    #    # Expected homogeneous transformation matrix
    #    self.Tsd = np.block([[desired_rot, desired_p], [0, 0, 0, 1]])

    #    # inverse kinematics
    #    self.compute_ik()

    # def Exponential_calculation(self,q,a,rot,s):
    #     tf = {1:np.eye((4)), 2:np.eye((4)), 3:np.eye((4)), 4:np.eye((4)), 5:np.eye((4))}
    #     r_cross = {1: [0, 0, 0], 2: [0, 0, 0], 3: [0, 0, 0]} # change the name to p or translation
    #     T = np.eye(4,4)
    #     n = len(q)
    #     for ii in range(n-1,-1,-1):
    #         rot_hat = np.array([[0, -rot[ii][2], rot[ii][1]],
    #                 [rot[ii][2], 0, -rot[ii][0]],
    #                 [-rot[ii][1], rot[ii][0], 0]])
    #         e_rot_hat=np.eye(3,3)+rot_hat*np.sin(q[ii])+rot_hat@rot_hat*(1-np.cos(q[ii]))

    #         if ii>0:
    #             Sv = -np.transpose(np.cross(rot[ii][:], a[ii][:]))
    #         elif ii==0:
    #             Sv = [0, 0, 0]
            
    #         p = (np.eye((3))*q[ii]+(1-np.cos(q[ii]))*rot_hat+(q[ii]-np.sin(q[ii]))*rot_hat@rot_hat)@Sv
    #         p = p.reshape(3,1)
    #         e_zai = np.block([[e_rot_hat, p], [0, 0, 0, 1]])

    #         tf[ii+1] = e_zai
        
    #     return J

    def screw_axis_to_transformation_matrix(self, screw_axis, angle):
        # screw_axis should be np.array([sw1 sw2, sw3, sv1, sv2, sv3])
        assert len(screw_axis) == 6, "Input screw axis must have six components"

        # Extract rotational and translational components from the screw axis
        Sw = screw_axis[:3]
        Sv = screw_axis[3:]

        # Matrix form of the screw axis
        screw_matrix = np.zeros((4, 4))
        screw_matrix[:3, :3] = np.array([[ 0, -Sw[2], Sw[1]],
                                        [Sw[2], 0, -Sw[0]],
                                        [-Sw[1], 0, Sw[0], ]])
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
        p = np.array([[0, -r[2], r[1]],
                            [r[2], 0, -r[0]],
                            [-r[1], r[0], 0]])
        pR= p @ R
        ad_tsb = np.block([[R, np.zeros((3, 3))],
                        [pR, R]])
        
        JB = ad_tsb @ JS
        # Calculate the body jacobian
        # J = np.array([[, , , ],
        #                 [, , , ],
        #                 [, , , ],
        #                 [, , , ],
        #                 [, , , ],
        #                 [, , , ]])
        return JB
    
    def Space_Jacobian(self):
        a = self.a
        q = self.angle_guess
        # rot = np.array([self.S[0,:3], self.S[1, :3], self.S[2, :3], self.S[3, :3], self.S[4, :3], self.S[5, :3]])
        rot = np.array([self.S[0,:3], self.S[1, :3], self.S[2, :3], self.S[3, :3]])
        # ad = {1:np.eye((6)), 2:np.eye((6)), 3:np.eye((6)), 4:np.eye((6)), 5:np.eye((6))}
        # tf = {1:np.eye((4)), 2:np.eye((4)), 3:np.eye((4)), 4:np.eye((4)), 5:np.eye((4))}
        ad = {1:np.eye((6)), 2:np.eye((6)), 3:np.eye((6))}
        tf = {1:np.eye((4)), 2:np.eye((4)), 3:np.eye((4))}
        translation = {1: [0, 0, 0], 2: [0, 0, 0], 3: [0, 0, 0]} 
        T = np.eye(4,4)
        Ts_b = np.eye(4,4)
    
        n = len(q)
        for ii in range(n-1,-1,-1):
            rot_hat = np.array([[0, -rot[ii][2], rot[ii][1]],
                    [rot[ii][2], 0, -rot[ii][0]],
                    [-rot[ii][1], rot[ii][0], 0]])
            e_rot_hat=np.eye(3,3)+rot_hat*np.sin(q[ii])+rot_hat@rot_hat*(1-np.cos(q[ii]))

            if ii>0:
                Sv = -np.transpose(np.cross(rot[ii][:], a[ii][:]))
            elif ii==0:
                Sv = [0, 0, 0]
            
            p = (np.eye((3))*q[ii]+(1-np.cos(q[ii]))*rot_hat+(q[ii]-np.sin(q[ii]))*rot_hat@rot_hat)@Sv
            p = p.reshape(3,1)
            e_zai = np.block([[e_rot_hat, p], [0, 0, 0, 1]])
            T = e_zai@T

            tf[ii+1] = e_zai

        Ts_b = T*self.M # from end_effector to the base
        Tb_s = np.linalg.inv(Ts_b)
        r = tf[1][:3, 3]
        translation[1] = np.array([[0, -r[2], r[1]],
                            [r[2], 0, -r[0]],
                            [-r[1], r[0], 0]])
        ad[1] = np.block([[tf[1][:3, :3], np.zeros((3, 3))],
                            [translation[1]@tf[1][:3, :3], tf[1][:3, :3]]])
        
        tf12 = tf[1] @ tf[2]
        R = tf12[:3, :3]
        r = tf12[:3, 3]
        translation[2] = np.array([[0, -r[2], r[1]],
                            [r[2], 0, -r[0]],
                            [-r[1], r[0], 0]])
        pR= translation[2] @ R
        ad[2] = np.block([[R, np.zeros((3, 3))],
                        [pR, R]])
        
        tf13 = tf[1] @ tf[2] @ tf[3]
        R = tf13[:3, :3]
        r = tf13[:3, 3]
        translation[3] = np.array([[0, -r[2], r[1]],
                            [r[2], 0, -r[0]],
                            [-r[1], r[0], 0]])
        pR= translation[3] @ R
        ad[3] = np.block([[R, np.zeros((3, 3))],
                        [pR, R]])
        
        # tf14 = tf[1] @ tf[2] @ tf[3] @ tf[4]
        # R = tf14[:3, :3]
        # r = tf14[:3, 3]
        # translation[4] = np.array([[0, -r[2], r[1]],
        #                     [r[2], 0, -r[0]],
        #                     [-r[1], r[0], 0]])
        # pR= translation[4] @ R
        # ad[4] = np.block([[R, np.zeros((3, 3))],
        #                 [pR, R]])
        
        # tf15 = tf[1] @ tf[2] @ tf[3] @ tf[4] @ tf[5]
        # R = tf15[:3, :3]
        # r = tf15[:3, 3]
        # translation[5] = np.array([[0, -r[2], r[1]],
        #                     [r[2], 0, -r[0]],
        #                     [-r[1], r[0], 0]])
        # pR= translation[5] @ R
        # ad[5] = np.block([[R, np.zeros((3, 3))],
        #                 [pR, R]])
        
        J1 = self.S[:, 0].reshape(-1, 1)  # Ensure 6x1
        print("ad[1]", ad[1])
        print("S2", self.S[:, 1])
        J2 = ad[1] @ self.S[:, 1].reshape(-1, 1)
        J3 = ad[2] @ self.S[:, 2].reshape(-1, 1)
        J4 = ad[3] @ self.S[:, 3].reshape(-1, 1)
        # J5 = ad[4] @ self.S[:, 4].reshape(-1, 1)
        # J6 = ad[5] @ self.S[:, 5].reshape(-1, 1)
        # JS = np.hstack([J1, J2, J3, J4, J5, J6])
        JS = np.hstack([J1, J2, J3, J4])

        
        return JS,  Ts_b, Tb_s

    def num_IK(self, Tsd, InitGuess):
        InitGuess = self.angle_guess
        for i in range(100):
            # Calculate the end-effector transform (Tsb) evaluated at the InitGuess using the helper functions that you wrote at the beginning.
            JS, Ts_b, Tb_s = self.Space_Jacobian()
            Tbd = np.linalg.inv(Ts_b) @ self.Tsd
            
            # Compute the body twist
            matrix_Vb = logm(Tbd)
            Vb = self.twist_vector_from_twist_matrix(matrix_Vb) 
            print("Vb", Vb)

            # Compute new angles
            JB = self.body_jacobian(JS, Tb_s)
            # self.get_logger().info(f"Body Jacobian is {JB @ np.transpose(JB)}")
            print("JB", JB)
            # JB_pseudoinv = np.transpose(JB) @ np.linalg.inv(JB @ np.transpose(JB)) #6-by-6 matrix
            JB_pseudoinv = np.linalg.pinv(JB)
            print("JB_pseudoinv", JB_pseudoinv)
            NewGuess = InitGuess+JB_pseudoinv @ Vb
            print("new_guess", NewGuess)
            print(f"Iteration number: {i} \n")

            # Check if you're done and update initial guess
            if(np.linalg.norm(abs(NewGuess-InitGuess)) <= 0.001):
                self.angle_guess = NewGuess
                # return [NewGuess[0], NewGuess[1], NewGuess[2], NewGuess[3], NewGuess[4], NewGuess[5], NewGuess[6]] 
                return [NewGuess[0], NewGuess[1], NewGuess[2], NewGuess[3]] 
            else:
                InitGuess = NewGuess
        print('Numerical solution failed!!')
        
        # joint_state = JointState()
        # joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        # joint_state.position = NewGuess
        # self.publisher_.publish(joint_state)
        # self.get_logger().info('JointState Publisher has been started.')
       
    
# sample code 

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




