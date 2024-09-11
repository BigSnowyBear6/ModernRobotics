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

    bot.arm.set_single_joint_position(joint_name='waist', position=1.57)
    bot.arm.go_to_home_pose()

    bot.arm.set_single_joint_position(joint_name='shoulder', position=-1.57)
    bot.arm.go_to_home_pose()

    bot.arm.set_single_joint_position(joint_name='elbow', position=0.8)
    bot.arm.go_to_home_pose()

    bot.arm.set_single_joint_position(joint_name='wrist_angle', position=-0.5)
    bot.arm.go_to_home_pose()

    bot.arm.go_to_sleep_pose()

    # bot.shutdown()


if __name__ == '__main__':
    main()