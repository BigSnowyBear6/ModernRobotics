# Copyright 2022 Trossen Robotics

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time

def main():

    # TODO: Define the joint angles in radians considering the joint limits
    joint_positions = [1.57, -0.785, 0, 0.785]

    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )
    bot.arm.go_to_home_pose()

    time.sleep(1)

    bot.arm.set_joint_positions(joint_positions)

    time.sleep(2)

    bot.arm.go_to_home_pose()

    time.sleep(1)
    
    bot.arm.go_to_sleep_pose()

    bot.shutdown()


if __name__ == '__main__':
    main()