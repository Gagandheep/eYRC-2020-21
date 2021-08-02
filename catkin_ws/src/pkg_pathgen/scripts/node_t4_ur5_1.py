#!/usr/bin/env python

from warehouse.warehouse import *


def main():
    """
    Main function
    """

    global color_id

    # Start a ROS node with name 'node_t3_ur5_1_pick_place'
    rospy.init_node('node_t3_ur5_1', anonymous=True)

    # Create an object of the class UR5_1
    ur5_1 = UR5('ur5_1')
    # Create an object of the class Camera2D
    cam2d = Camera2D()

    rospy.sleep(1)

    # Get colors of packages from shelf
    colors = cam2d.get_colors()
    # Create an object of the class Shelf
    shelf = Shelf(colors)

    # Go to default pose
    play_traj(ur5_1, 'ur5_1_default')
    ur5_1.move_group.stop()

    # ur5_1.move_group.set_joint_value_target(
    #     ur5_1.joint_angles_conveyor)
    # for _ in range(10):
    #     plan1 = ur5_1.move_group.plan()
    #     f1 = ur5_1.move_group.execute(plan1, wait=True)
    #     ur5_1.move_group.stop()
    #     if f1:
    #         break

    poses = list()

    for pack in [9, 10, 11]:
        # Get package data from shelf
        package = shelf.get(pack)
        x, y = package.position

        flag = False
        # Move EE to Package
        while not flag:
            t1 = rospy.Time.now()
            flag = play_traj(ur5_1, 'ur5_1_default_{}{}'.format(x, y))
            t2 = rospy.Time.now()
        ur5_1.move_group.stop()

        # Activate Vacuum Gripper
        ur5_1.activate_vacuum_gripper()

        rospy.sleep(0.5)
        poses.append(ur5_1.move_group.get_current_joint_values())
        rospy.sleep(0.5)

        flag = False
        # Move EE to Conveyor
        while not flag:
            t3 = rospy.Time.now()
            flag = play_traj(ur5_1, 'ur5_1_{}{}_default'.format(x, y))
            t4 = rospy.Time.now()
        ur5_1.move_group.stop()

        # noinspection PyUnboundLocalVariable
        dt1 = t2 - t1
        # noinspection PyUnboundLocalVariable
        dt2 = t4 - t3

        append_to_file('durs.txt', '{}\t{}\t{}\n'.format(-1, pack, dt1))
        append_to_file('durs.txt', '{}\t{}\t{}\n'.format(pack, -1, dt2))
        rospy.sleep(1)

        # Wait for the conveyor to move to prevent stacking of packages
        # while len(cam_logical.get_packages()) != 1:
        #     pass

        # Deactivate Vacuum Gripper
        ur5_1.deactivate_vacuum_gripper()

        # color_id = package.color_id

    # Wait for last package data to be published
    # while color_id is not None:
    #     pass

    print poses

    # Delete all objects
    del ur5_1
    del shelf
    del cam2d

    # rospy.spin()


if __name__ == '__main__':
    color_id = None
    main()
