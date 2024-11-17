#!/usr/bin/env python3 
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
    
    # Create joint angular velocity publishers (after editing the yaml file)
    self.a_pub = self.create_publisher(JointGroupCommand, 'px100/commands/joint_group', 1) 

    self.publisher_vel = self.create_publisher(
      Twist, 
      'end_eff_vel', 
      1)
    
    # Velocity error publisher
    self.publisher_vel_err = self.create_publisher(
      Twist, 
      'vel_err', 
      1)

    # Call on_timer function on a set interval
    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.on_timer)
     
    # Past variables' initialization (explain in your report why these are required)
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
      JointState, 
      'px100/joint_states', 
      self.listener_callback, 
      1)
    self.subscription # prevent unused variable warning

    # Create joint position variable (initialization)
    self.angles = [0.0, 0.0, 0.0, 0.0]

  def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.angles = [data.position[0], data.position[1], data.position[2], data.position[3]]

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
    homogeneous_matrix = self.a2rot(trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w)   
    homogeneous_matrix[0,3] = trans.transform.translation.x; homogeneous_matrix[1,3] = trans.transform.translation.y
    homogeneous_matrix[2,3] = trans.transform.translation.z

    # Compute the time derivative of T using numerical differentiation (explain this in your report)
    # print(f'homogeneous_matrixis {homogeneous_matrix}')
    homogeneous_matrix_deriv = (homogeneous_matrix-self.homogeneous_matrix_old) / 0.1 # Transformation derivative
    self.homogeneous_matrix_old = homogeneous_matrix # Update your old records
    # print(homogeneous_matrix[0:3])
    homogeneous_matrix_inv = np.linalg.inv(homogeneous_matrix)
    # Compute the matrix form of the twist (write the math equations that you used to complete this in your report)
    # print(f'homogeneous_matrix_deriv {homogeneous_matrix_deriv}')
    # print(f'homogeneous_matrix_inv {homogeneous_matrix_inv}')
    # print(f'homogeneous_matrix_inv position {np.transpose(homogeneous_matrix[0:3])}')
    vel_brack = homogeneous_matrix_deriv@homogeneous_matrix_inv
    # print(f'vel_brack is {vel_brack}')
    ang_vel = np.array([vel_brack[2,1], vel_brack[0,2], vel_brack[1,0]])# Angular velocity vector of gripper w.r.t world frame
    trans_vel =  np.array([vel_brack[0,3], vel_brack[1,3], vel_brack[2,3]])# Translational velocity vector of a point on the origin of the {s} frame expressed w.r.t world frame
    # print(f'ang_vel is {ang_vel}')
    # print(f'trans_vel is {trans_vel}')
    # Publish the velocity message
    vel_msg = Twist()

    vel_msg.linear.x = trans_vel[0]
    vel_msg.linear.y = trans_vel[1]
    vel_msg.linear.z = trans_vel[2]
    vel_msg.angular.x = ang_vel[0]
    vel_msg.angular.y = ang_vel[1]
    vel_msg.angular.z = ang_vel[2]
    self.publisher_vel.publish(vel_msg) 

    # Publish velocity commands
    t = trans.header.stamp.sec # Time stamp in seconds
    a_msg = JointGroupCommand()                 
    a_msg.name = 'arm'
    
    # Robot motion commands
    # Feel free to design your own dance moves!
    if t-self.ti <= 1:
        # Stretch the arm a bit
        a_msg.cmd = [0.0, 1.0, -1.0, 0.0] # Initial velocity (rad/sec.)
    elif t-self.ti > 1 and t-self.ti <= 3:
        # Freeze
        a_msg.cmd = [0.0, 0.0, 0.0, 0.0] # Initial velocity (rad/sec.)
    elif t-self.ti > 3 and t-self.ti <= 10:
        # Dance --> Feel free to design your own dance moves
        a_msg.cmd = [0.3, 0.0, 2*sin(3.7*(self.get_clock().now().nanoseconds / 1e9)), 0.0] # Initial velocity (rad/sec.)
    elif t-self.ti > 10 and t-self.ti <= 24:
    # Dance --> Feel free to design your own dance moves
         a_msg.cmd = [-0.3, 0.0, 2*sin(3.7*(self.get_clock().now().nanoseconds / 1e9)), 0.0]
    elif t-self.ti > 24 and t-self.ti <= 31:
    # Dance --> Feel free to design your own dance moves
         a_msg.cmd = [0.3, 0.0, 2*sin(3.7*(self.get_clock().now().nanoseconds / 1e9)), 0.0]
    else:
        # Freeze again
        a_msg.cmd = [0.0, 0.0, 0.0, 0.0] # Initial velocity (rad/sec.)

    # Publish velocity commands
    self.a_pub.publish(a_msg)
    
    s1 = np.array([0, 0, 1, 0, 0, 0])
    s2 = np.array([0, 1, 0, -89.45, 0, 0])
    s3 = np.array([0, 1, 0, -189.45, 0, 35])
    s4 = np.array([0, 1, 0, -189.45, 0, 135])
    s = np.stack([s1, s2, s3, s4], axis=1)
    a = [[0, 0, 0],[0, 0, 89.45],[35, 0, 189.45],[135, 0, 189.45]]
    rot = [[0, 0, 1],[0, 1, 0],[0, 1, 0],[0, 1, 0]]
    self.J =self.Jacobian(self.angles,a,rot,s)
    self.get_logger().info("Jacobian is {self.J}")
    vel_from_jac = self.J @ np.array([[a_msg.cmd[0]],
                                      [a_msg.cmd[1]],
                                      [a_msg.cmd[2]],
                                      [a_msg.cmd[3]]])
    
     # Publish the velocity error message
    vel_err_msg = Twist()
    print('trans_vel[0]', trans_vel[0])
    print('vel_from_jac[3]', vel_from_jac[3])
    print('trans_vel[0]-vel_from_jac[3]', float((trans_vel[0] - vel_from_jac[3])[0]))
    vel_err_msg.linear.x = float((trans_vel[0] - vel_from_jac[3])[0])
    vel_err_msg.linear.y = float((trans_vel[1]-vel_from_jac[4])[0])
    vel_err_msg.linear.z = float((trans_vel[2]-vel_from_jac[5])[0])
    vel_err_msg.angular.x = float((ang_vel[0]-vel_from_jac[0])[0])
    vel_err_msg.angular.y = float((ang_vel[1]-vel_from_jac[1])[0])
    vel_err_msg.angular.z = float((ang_vel[2]-vel_from_jac[2])[0])
    self.publisher_vel_err.publish(vel_err_msg)

  def a2rot(self, q0, q1, q2, q3):
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
  
  def Jacobian(self,q,a,rot,s):
    ad = {1:np.eye((6)), 2:np.eye((6)), 3:np.eye((6))}
    tf = {1:np.eye((4)), 2:np.eye((4)), 3:np.eye((4))}
    r_cross = {1: [0, 0, 0], 2: [0, 0, 0], 3: [0, 0, 0]}
    T = np.eye(4,4)
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

      tf[ii+1] = e_zai
    
    r = tf[1][:3, 3]
    r_cross[1] = np.array([[0, -r[2], r[1]],
                        [r[2], 0, -r[0]],
                        [-r[1], r[0], 0]])
    ad[1] = np.block([[tf[1][:3, :3], np.zeros((3, 3))],
                        [r_cross[1]@tf[1][:3, :3], tf[1][:3, :3]]])
    
    tf12 = tf[1] @ tf[2]
    R = tf12[:3, :3]
    r = tf12[:3, 3]
    r_cross[2] = np.array([[0, -r[2], r[1]],
                        [r[2], 0, -r[0]],
                        [-r[1], r[0], 0]])
    pR= r_cross[2] @ R
    ad[2] = np.block([[R, np.zeros((3, 3))],
                      [pR, R]])
    
    tf13 = tf[1] @ tf[2] @ tf[3]
    R = tf13[:3, :3]
    r = tf13[:3, 3]
    r_cross[3] = np.array([[0, -r[2], r[1]],
                        [r[2], 0, -r[0]],
                        [-r[1], r[0], 0]])
    pR= r_cross[3] @ R
    ad[3] = np.block([[R, np.zeros((3, 3))],
                      [pR, R]])
    
    J1 = s[:, 0].reshape(-1, 1)  # Ensure 6x1
    J2 = ad[1] @ s[:, 1].reshape(-1, 1)
    J3 = ad[2] @ s[:, 2].reshape(-1, 1)
    J4 = ad[3] @ s[:, 3].reshape(-1, 1)
    J = np.hstack([J1, J2, J3, J4])
    return J

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
