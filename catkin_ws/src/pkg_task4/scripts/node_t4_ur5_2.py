#!/usr/bin/env python

from warehouse.warehouse import *


def subscriber_callback(msg):
    """
    Subscriber Callback function to read color and offset data
    """
    global bin_ids
    global offsets

    bin_ids.append(msg.color)
    offsets.append(msg.offset)


def cam_callback(obj, conveyor, ur5):
    """
    Camera Callback function to control conveyor speed
    """
    if not kill_callback:

        if not ur5.attached:
            for box in obj.get_packages():
                if box.pose.position.y < 0.03:
                    conveyor.stop()
                    obj.update_overlap(box)
                    return
                elif box.pose.position.y < 0.07:
                    conveyor.move(11)
                    obj.update_overlap(box)
                    return
                elif box.pose.position.y < 0.1:
                    conveyor.move(50)
                    obj.update_overlap(box)
                    return

        conveyor.move(100)
        obj._overlap = 0.0


def main():
    """
    Main function
    """
    global bin_ids
    global offsets
    global kill_callback

    # Start a ROS node with name 'node_t3_ur5_1_pick_place'
    rospy.init_node('node_t3_ur5_2', anonymous=True)

    # Subscriber to listen to package data sent by node_t4_ur5_1
    sub_handler = rospy.Subscriber('/package_data', PackageInfo,
                                   subscriber_callback)

    # Create an object of the class UR5_2
    ur5_2 = UR5('ur5_2')
    # Create an object of the class Conveyor
    conveyor = Conveyor()
    # Create an object of the class VacuumSensor
    vac_sensor = VacuumSensor(
        '/eyrc/vb/ur5_2/vacuum_gripper/logical_camera/ur5_2')
    # Create an object of the class LogicalCamera
    cam_logical = LogicalCamera('/eyrc/vb/logical_camera_2', ur5_2,
                                cam_callback, conveyor, ur5_2)

    # Joint angles for moving the EE above the Conveyor
    joint_angles_conveyor = [
        0.13694346680287595,
        -2.442564757819942,
        -1.0171948425452548,
        -1.2517722106942388,
        1.5709108345485774,
        0.13736034068502523]

    # Joint angles for moving the EE above the Red bin
    joint_angles_red = [
        0.15676897704131143 - math.pi / 2 + math.pi / 25,
        -2.117432695000222,
        -1.3140512668750404,
        -1.2809050098566246,
        1.5707961002699218,
        0.15676897682691493]

    # Joint angles for moving the EE above the Yellow bin
    joint_angles_yellow = [
        0.15676897704131143 - math.pi + math.pi / 15,
        -2.117432695000222,
        -1.3140512668750404,
        -1.2809050098566246,
        1.5707961002699218,
        0.15676897682691493]

    # Joint angles for moving the EE above the Green bin
    joint_angles_green = [
        0.15676897704131143 + math.pi / 2 - math.pi / 18,
        -2.117432695000222,
        -1.3140512668750404,
        -1.2809050098566246,
        1.5707961002699218,
        0.15676897682691493]

    bin_joint_angles = [joint_angles_red,
                        joint_angles_yellow,
                        joint_angles_green]

    # Move EE above the Conveyor
    ur5_2.set_joint_angles(joint_angles_conveyor)
    ur5_2.move_group.stop()

    # Pre-compute the joint angles for all the positions on the conveyor
    ur5_2.calculate_pickup_angles()

    # Pick and drop the boxes in their respective bins
    done = []
    i = 0

    while True:
        if len(offsets) == 0:
            ur5_2.set_joint_angles(joint_angles_conveyor)
            ur5_2.move_group.stop()

        # Wait for package data
        while len(offsets) == 0:
            pass

        # Apply offset to EE's position
        ur5_2.go_to_position(offsets.pop(0))
        ur5_2.move_group.stop()

        while len(cam_logical.get_packages()) == 0:
            pass

        packages = cam_logical.get_packages()

        t = rospy.Time.now()
        # Plan the path
        ur5_2.set_joint_angles(bin_joint_angles[bin_ids[0]], mode=2)
        ur5_2.move_group.stop()

        for package in packages:
            if package.type not in done:
                done.append(package.type)

                while not vac_sensor.attachable or cam_logical.overlap < 0.7:
                    pass

                # Activate Vacuum Gripper
                ur5_2.activate_vacuum_gripper()

                # Execute the planned path
                ur5_2.set_joint_angles(bin_joint_angles[bin_ids.pop(0)],
                                       mode=1)
                ur5_2.move_group.stop()

                # Deactivate Vacuum Gripper
                ur5_2.deactivate_vacuum_gripper()

                i += 1

        if i == 9:
            break

    # Move EE to Conveyor
    ur5_2.set_joint_angles(joint_angles_conveyor)
    ur5_2.move_group.stop()

    # Kill Callback controlling the conveyor and stop the conveyor
    kill_callback = True
    conveyor.stop()

    # Delete all objects
    del cam_logical
    del vac_sensor
    del conveyor
    del ur5_2

    rospy.spin()


if __name__ == '__main__':
    bin_ids = []
    offsets = []
    kill_callback = False
    main()
