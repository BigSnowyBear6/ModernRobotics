# Copyright 2022 Trossen Robotics 

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

def main():
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )

    bot.arm.go_to_home_pose()
    
    bot.arm.set_single_joint_position(joint_name='elbow', position=np.pi/4.0)

    bot.gripper.set_pressure(1.0)
    bot.gripper.grasp(2.0)

    bot.arm.set_single_joint_position(joint_name='elbow', position=-np.pi/4.0)
    bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/4.0)
    bot.arm.set_single_joint_position(joint_name='elbow', position=np.pi/4.0)
    
    bot.gripper.release(2.0)
    bot.arm.set_single_joint_position(joint_name='elbow', position=-np.pi/4.0)


    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__ == '__main__':
    main()
