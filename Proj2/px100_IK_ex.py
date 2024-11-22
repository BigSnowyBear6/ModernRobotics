# px100_jac_ex.py
#!/usr/bin/env python3 
 
# Import the ROS client library for Python 
import rclpy 
 
# Enables the use of rclpy's Node class
from rclpy.node import Node 
 
# Base class to handle exceptions
from tf2_ros import TransformException 
 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
 
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener 

# We need the joint command message to send velcoity commands to joints
from interbotix_xs_msgs.msg import JointGroupCommand
 
# Import the 'Twist' message type from the 'geometry_msgs' package
# 'Twist' represents linear and angular velocities in a 3D space
from geometry_msgs.msg import Twist

# Include joint state message
from sensor_msgs.msg import JointState
 
# Math library
import math 
from math import sin, cos, pi

# Numpy library
import numpy as np

class FrameListener(Node):
  """
  Subclass of the Node class.
  The class listens to coordinate transformations and 
  publishes the end-effector velocity at a specific time interval.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    
    # Initiate the Node class's constructor and give it a name
    super().__init__('map_base_link_frame_listener')
 
    # Declare and acquire `target_frame` parameter
    self.declare_parameter('target_frame', 'px100/ee_gripper_link')
    self.target_frame = self.get_parameter(
      'target_frame').get_parameter_value().string_value
 
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
      
    # Velocity publisher
   
    self.publisher_vel = self.create_publisher(
              Twist, 
      'end_eff_vel', 
      1)
    
     # Velocity error publisher
    self.publisher_vel_err = self.create_publisher(
               Twist , 
      'vel_err', 
      1)
 
    # Call on_timer function on a set interval
    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.on_timer)
     
    # Past variables' initialization
    self.homogeneous_matrix_old = np.zeros((4, 4)); self.homogeneous_matrix_old[3, 3] = 1.0 # Past homogeneous matrix
    self.ti = self.get_clock().now().nanoseconds / 1e9 # Initial time
    
  
    # Define your jacobian matrix which is dependent on joint positions (angles) (make sure to show your calculation in your report)
    # all zero elements of the matrix should be calculated and entered in this matrix as a function of joint angles
    self.J = np.array([[0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0, 0.0]]) # Iinitial jacobian
    
    # Create the subscriber
    self.subscription = self.create_subscription(
                      JointState , 
      'px100/joint_states', 
      self.listener_callback, 
      1)
    self.subscription # prevent unused variable warning

    # Create joint position variable (initialization)
    self.angles = [0.0, 0.0, 0.0, 0.0]
    
  def on_timer(self):
    """
    Callback function.
    This function gets called at the specific time interval.
    """
    # Store frame names in variables that will be used to
    # compute transformations
    from_frame_rel = self.target_frame
    to_frame_rel = 'px100/base_link'
   
    trans = None
     
    try:
      now = rclpy.time.Time()
      trans = self.tf_buffer.lookup_transform(
                  to_frame_rel,
                  from_frame_rel,
                  now)
    except TransformException as ex:
      self.get_logger().info(
        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
      return
       
    # Get the homogeneous matrix (explain these in your report)
    homogeneous_matrix = self.quatRot(trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w)   
    homogeneous_matrix[0,3] = trans.transform.translation.x; homogeneous_matrix[1,3] = trans.transform.translation.y
    homogeneous_matrix[2,3] = trans.transform.translation.z

    # Compute the time derivative of T using numerical differentiation (explain this in your report)
    homogeneous_matrix_deriv = ( homogeneous_matrix-self.homogeneous_matrix_old) / 0.1 # Transformation derivative
    self.homogeneous_matrix_old = homogeneous_matrix # Update your old records
    homogeneous_matrix_inv = np.linalg.inv(homogeneous_matrix)#np.block([[np.transpose(homogeneous_matrix)],[-np.transpose(homogeneous_matrix)@np.array([[trans.transform.rotation.x],[trans.transform.rotation.y],[trans.transform.rotation.z]])],[0,0,0,1]])
    # Compute the matrix form of the twist (write the math equations that you used to complete this in your report)
    vel_brack = homogeneous_matrix_deriv@homogeneous_matrix_inv
    ang_vel =  np.array([vel_brack[2,1],vel_brack[0,2],vel_brack[1,0]])# Angular velocity vector of gripper w.r.t world frame
    trans_vel =  np.array([vel_brack[0,3],vel_brack[1,3],vel_brack[2,3]])# Translational velocity vector of a point on the origin of the {s} frame expressed w.r.t world frame

    # Publish the velocity message
    vel_msg = Twist()
    vel_msg.linear.x = trans_vel[0]
    vel_msg.linear.y = trans_vel[1]
    vel_msg.linear.z = trans_vel[2]
    vel_msg.angular.x = ang_vel[0]
    vel_msg.angular.y = ang_vel[1]
    vel_msg.angular.z = ang_vel[2]
    self.publisher_vel.publish(vel_msg) 
    
    
    
    # Compute twist using jacobian
    S1 = np.transpose(np.array([0, 0 ,1, 0,0,0]))
    S2 = np.transpose(np.array([0, 1 ,0, -0.08945,0,0]))
    S3 = np.transpose(np.array([0, 1 ,0, -0.18945,0,0.035]))
    S4 = np.transpose(np.array([0, 1 ,0, -0.18945,0,0.135]))
    # self.angles
    
    self.J = 
    
    
    vel_from_jac = self.J @ np.array([[0.0],
                                      [0.0],
                                      [0.0],
                                      [0.0]])
    
    # Publish the velocity error message
    vel_err_msg = 
    vel_err_msg.linear.x = 
    vel_err_msg.linear.y =
    vel_err_msg.linear.z = 
    vel_err_msg.angular.x = 
    vel_err_msg.angular.y =
    vel_err_msg.angular.z = 
    self.publisher_vel_err.publish(vel_err_msg)
  
  
  def quatRot(self, q0, q1, q2, q3):
    """
    Convert a quaternion into a rotation matrix

    """
   
    # Calculate the rotation matrix

    rotation_matrix = np.array([[np.square(q0)+np.square(q1)-np.square(q2)-np.square(q3), 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3)],
                               [2*(q1*q2+q0*q3), np.square(q0)-np.square(q1)+np.square(q2)-np.square(q3), 2*(q2*q3-q0*q1)],
                               [2*(-q0*q2+q1*q3), 2*(q2*q3+q0*q1), np.square(q0)-np.square(q1)-np.square(q2)+np.square(q3)]])



    # Create a 4x4 homogeneous transformation matrix with zero translational elements (explain in the report why we should do this?)
    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    return homogeneous_matrix
  

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  frame_listener_node = FrameListener()
  
  # Spin the node so the callback function is called.
  # Publish any pending messages to the topics.
  try:
    rclpy.spin(frame_listener_node)
  except KeyboardInterrupt:
    pass
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()