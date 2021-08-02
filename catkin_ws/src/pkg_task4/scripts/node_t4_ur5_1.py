#!/usr/bin/env python

from warehouse.warehouse import *


def cam_callback(obj, publisher):
    """
    Camera Callback function to publish color and offset data
    """
    global color_id

    for package in obj.get_packages():
        if color_id is not None and package.pose.position.z < 1.2:
            publisher.publish(color_id, package.pose.position.z)
            color_id = None


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
    # Start a publisher to publish package data
    package_info_publisher = PackageInfoPublisher('/package_data')
    # Create an object of the class LogicalCamera
    cam_logical = LogicalCamera('/eyrc/vb/logical_camera_1', ur5_1,
                                cam_callback, package_info_publisher)

    # Go to default pose
    ur5_1.play_trajectory('ur5_1_default')
    ur5_1.move_group.stop()

    rospy.sleep(2)

    # Get colors of packages from shelf
    colors = cam2d.get_colors()
    # Create an object of the class Shelf
    shelf = Shelf(colors)

    for pack in range(8, -1, -1):
        # Get package data from shelf
        package = shelf.get(pack)
        x, y = package.position

        # Move EE to Package
        while not ur5_1.play_trajectory('ur5_1_default_{}{}'.format(x, y)):
            pass
        ur5_1.move_group.stop()

        # Activate Vacuum Gripper
        ur5_1.activate_vacuum_gripper()

        # Move EE to Conveyor
        while not ur5_1.play_trajectory('ur5_1_{}{}_default'.format(x, y)):
            pass
        ur5_1.move_group.stop()

        # Wait for the conveyor to move to prevent stacking of packages
        while len(cam_logical.get_packages()) != 1:
            pass

        # Deactivate Vacuum Gripper
        ur5_1.deactivate_vacuum_gripper()

        color_id = package.color_id

    # Wait for last package data to be published
    while color_id is not None:
        pass

    # Delete all objects
    del cam_logical
    del ur5_1
    del shelf
    del cam2d

    rospy.spin()


if __name__ == '__main__':
    color_id = None

    try:
        main()
    except:
        cv2.destroyAllWindows()
