from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from proj2_1 import ourAPI
import numpy as np
from math import atan2, sin, cos, pi
# You can use the time library if you ever need to make some delay. For example: time.sleep(3) 
import time

# Start by defining some constants such as robot model, visual perception frames, basket transform, etc.
ROBOT_MODEL = 'px100'
ROBOT_NAME = ROBOT_MODEL
REF_FRAME = 'camera_color_optical_frame'
ARM_TAG_FRAME = f'{ROBOT_NAME}/ar_tag_link'
ARM_BASE_FRAME = f'{ROBOT_NAME}/base_link'
Td_release = np.array([[0,  1,  0,  0],
                    [0,  0,  -1,  -0.5*242.6],
                    [-1,  0,  0,  8*15],
                    [0,  0,  0,  1]]) 

R_operator =  np.array([[0,  0,  1],
                    [0,  1,  0],
                    [-1,  0,  0]]) 

def main():
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )
    
    pcl = InterbotixPointCloudInterface(node_inf=bot.core.robot_node)
    armtag = InterbotixArmTagInterface(
        ref_frame=REF_FRAME,
        arm_tag_frame=ARM_TAG_FRAME,
        arm_base_frame=ARM_BASE_FRAME,
        node_inf=bot.core.robot_node
    )
    my_api = ourAPI()

    # set initial arm and gripper pose
    bot.arm.go_to_sleep_pose()
    bot.gripper.release()

    # get the ArmTag pose
    armtag.find_ref_to_arm_base_transform() 

    # get the cluster positions
    # sort them from max to min 'x' position w.r.t. the ARM_BASE_FRAME
    success, clusters = pcl.get_cluster_positions(
        ref_frame=ARM_BASE_FRAME,
        sort_axis='x',
        reverse=True
    )
    
    lower = [[100, 0, 0],[15, 30, 30],[100, 100,10]]
    upper = [[255, 75, 55],[45, 255, 100],[255,255,100]]

    if success:
        bot.arm.go_to_home_pose()
        # pick up all the objects and drop them in a basket (note: you can design your own experiment)
        for cluster in clusters:
            print(f"cluster color {cluster['color']}")
            if all(lower[1][i] <= cluster['color'][i] <= upper[1][i] for i in range(3)):
                # Get the first cube location
                x, y, z = cluster['position']
                z = z + 0.02 # Fingers link offset (change this offset to match your experiment)
                y = y 
                x = x - 0.01

                # Go on top of the selected cube
                theta_base = np.arctan2(y,x) # Waist angle offset
                
                # desired pose
                R_grasp = np.array([[np.cos(theta_base),  -np.sin(theta_base),  0],
                        [np.sin(theta_base),  np.cos(theta_base),  0],
                        [0,  0,  1]]) @ R_operator
                p = np.array([[x*1000], [y*1000], [z*1000]])
                Td_grasp = np.block([[R_grasp, p], [0,0,0,1]])

                joint_positions=np.array([theta_base,0,0,0])
                bot.arm.set_joint_positions(joint_positions)
                time.sleep(1)
                
                joint_positions = my_api.num_IK(Td_grasp, np.array([1.5,0,0,0])) 
                bot.arm.set_joint_positions(np.append(theta_base,joint_positions[1:]))  
                

                bot.gripper.grasp(2.0) 

                joint_positions=np.array([theta_base,0,0,0])
                bot.arm.set_joint_positions(joint_positions)
                time.sleep(1)

                bot.arm.go_to_home_pose()
                
                joint_positions = my_api.num_IK(Td_release, np.array([0,0,0,0]))
                bot.arm.set_joint_positions(joint_positions)

                bot.gripper.release(2.0)
                
                bot.arm.set_joint_positions(np.array([joint_positions[0], 0, 0, 0]))
                time.sleep(1)
                
                bot.arm.go_to_home_pose()


    else:
        print('Could not get cluster positions.')
    
        # Go to sleep
    bot.arm.go_to_sleep_pose()
    bot.gripper.release(1.0)
    bot.shutdown()


if __name__ == '__main__':
    main()