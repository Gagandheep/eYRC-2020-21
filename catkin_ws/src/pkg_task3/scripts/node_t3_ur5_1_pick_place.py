#!/usr/bin/env python

import math
import sys
import rospy
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsg, conveyorBeltPowerMsgRequest
from pkg_vb_sim.msg import LogicalCameraImage


class Conveyor:

    def __init__(self):
        """
        class init
        """

        conveyor_service_name = '/eyrc/vb/conveyor/set_power'

        rospy.wait_for_service(conveyor_service_name)
        self.conveyor_service = rospy.ServiceProxy(conveyor_service_name,
                                                   conveyorBeltPowerMsg)

        self._is_moving = False
        self._current_power = 0

    @property
    def is_moving(self):
        """
        Class property
        Returns moving state of the conveyor
        """

        return self._is_moving

    def move(self, power=100):
        """
        Function to move the conveyor with the specified power
        """

        # Perform check to avoid setting same power repeatedly which may lead
        # to crashes
        if power != self._current_power:
            self._current_power = power
            self._is_moving = power != 0
            req = conveyorBeltPowerMsgRequest(power=power)
            self.conveyor_service(req)

    def stop(self):
        """
        Function
        to stop the conveyor
        """
        self.move(power=0)


class VacuumSensor:

    def __init__(self):
        """
        class init
        """

        rospy.Subscriber("/eyrc/vb/ur5_1/vacuum_gripper/logical_camera/ur5_1",
                         LogicalCameraImage,
                         self.attachable_callback)

        self._attachable = False

    def attachable_callback(self, data):
        """
        Callback function to set the class property _attachable
        """

        for model in data.models:
            name_model = model.type

            lst_name_model = name_model.split('n')

            if lst_name_model[0] == 'package':
                self._attachable = True
                break

        else:
            self._attachable = False

    @property
    def attachable(self):
        """
        Class property
        Returns attachable state of the gripper
        """

        return self._attachable


class LogicalCamera:

    def __init__(self, conveyor, ur5):
        """
        class init
        """

        self._models = []
        self.conveyor = conveyor
        self._overlap = 0.0

        self.subscriber = rospy.Subscriber('/eyrc/vb/logical_camera_2',
                                           LogicalCameraImage,
                                           self.subscriber_callback)

        self.ur5 = ur5
        self.move_group = ur5.move_group
        self.observed_box = ''
        self._kill_callback = False

    def kill_callback(self):
        """
        Function to permanently stop the conveyor
        """

        self._kill_callback = True

    def subscriber_callback(self, data):
        """
        Callback function to control the conveyor speed
        """

        self._models = data.models
        if self.ur5.attached and self.observed_box in self.get_types() \
                or self._kill_callback:
            self.conveyor.stop()
            return

        # Set conveyor speed based on the position of the packages
        for box in self.get_packages():
            if box.pose.position.y < 0.03:
                self.conveyor.stop()
                self.observed_box = box.type
                self._update_overlap(box)
                return
            elif box.pose.position.y < 0.07:
                self.conveyor.move(11)
                self.observed_box = box.type
                self._update_overlap(box)
                return
            elif box.pose.position.y < 0.1:
                self.conveyor.move(50)
                self.observed_box = box.type
                self._update_overlap(box)
                return

        self.conveyor.move(100)
        self._overlap = 0.0

    def _update_overlap(self, box):
        """
        Function to calculate the overlap between the positions
        of the package and the vacuum gripper
        Value range: [0, 1]
        """

        ee_pose = self.move_group.get_current_pose().pose
        ee_x = ee_pose.position.x
        ee_y = ee_pose.position.y
        box_pose = box.pose
        box_z = box_pose.position.z - 0.8
        box_y = box_pose.position.y

        self._overlap = min(
            max(1 - math.sqrt((ee_x - box_z) ** 2 + (ee_y - box_y) ** 2) * 5,
                0), 1)

    def get_models(self):
        """
        Function to return the models in the FOV of the camera
        """

        return self._models

    @property
    def overlap(self):
        """
        Class property
        Returns the overlap value
        """

        return self._overlap

    def get_packages(self):
        """
        Function to return the packages in the FOV of the camera
        """

        packages = []
        for model in self._models:
            try:
                if model.type != 'ur5':
                    packages.append(model)

            except:
                pass

        return packages

    def get_types(self):
        """
        Function to return the names of the packages in the FOV of the camera
        """

        types = []
        for model in self._models:
            try:
                if model.type != 'ur5':
                    types.append(model.type)

            except:
                pass

        return types


class UR5_1:

    def __init__(self):
        """
        class init
        """

        self.commander = moveit_commander.roscpp_initialize(sys.argv)
        self.move_group = moveit_commander.MoveGroupCommander(
            'ur5_1_planning_group')
        self.robot = moveit_commander.RobotCommander()
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1)

        self.execute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory',
            moveit_msgs.msg.ExecuteTrajectoryAction)
        self.execute_trajectory_client.wait_for_server()

        vacuum_service_name = '/eyrc/vb/ur5_1/activate_vacuum_gripper'

        rospy.wait_for_service(vacuum_service_name)
        self.vacuum_service = rospy.ServiceProxy(vacuum_service_name,
                                                 vacuumGripper)

        self.pickup_angles = dict()

        # Joint angles for default pose
        self.joint_angles_conveyor = [
            0.13694346680287595,
            -2.442564757819942,
            -1.0171948425452548,
            -1.2517722106942388,
            1.5709108345485774,
            0.13736034068502523]

        # Joint angles for moving the EE above the Red bin
        self.joint_angles_red = [
            0.15676897704131143 - math.pi / 2 + math.pi / 25,
            -2.117432695000222,
            -1.3140512668750404,
            -1.2809050098566246,
            1.5707961002699218,
            0.15676897682691493]

        # Joint angles for moving the EE above the Green bin
        self.joint_angles_green = [
            0.15676897704131143 - math.pi + math.pi / 15,
            -2.117432695000222,
            -1.3140512668750404,
            -1.2809050098566246,
            1.5707961002699218,
            0.15676897682691493]

        # Joint angles for moving the EE above the Blue bin
        self.joint_angles_blue = [
            0.15676897704131143 + math.pi / 2 - math.pi / 18,
            -2.117432695000222,
            -1.3140512668750404,
            -1.2809050098566246,
            1.5707961002699218,
            0.15676897682691493]

        self.bin_joint_angles = [self.joint_angles_red,
                                 self.joint_angles_green,
                                 self.joint_angles_blue]

        # Move the EE to its default position
        self.set_joint_angles(self.joint_angles_conveyor)

        # Save default pose
        self.default_pose = copy.deepcopy(
            self.move_group.get_current_pose().pose)

        self._attached = False

    @property
    def attached(self):
        """
        Class property
        Returns attached state vacuum gripper
        """

        return self._attached

    def activate_vacuum_gripper(self):
        """
        Function to activate the vacuum gripper
        """

        # Call ROS service to activate the vacuum gripper
        req = vacuumGripperRequest(activate_vacuum_gripper=True)
        self.vacuum_service(req)

        self._attached = True

    def deactivate_vacuum_gripper(self):
        """
        Function to deactivate the vacuum gripper
        """

        # Call ROS service to deactivate the vacuum gripper
        req = vacuumGripperRequest(activate_vacuum_gripper=False)
        self.vacuum_service(req)

        self._attached = False

    def calculate_pickup_angles(self, step=0.01,
                                longitudinal_range=(-0.1, 0.2)):
        """
        Function to calculate and store the joint angles for all the positions
        on the conveyor along the z-axis of the logical camera
        """

        self.pickup_angles = dict()

        i = longitudinal_range[0]
        while i <= longitudinal_range[1]:
            self.pickup_angles[i] = self.translation_angles(self.default_pose,
                                                            i, 0, 0)
            i = round(i + step, 2)

    def set_joint_angles(self, arg_list_joint_angles, wait=True):
        """
        Function to set the joint angles of the UR5 arm
        """

        list_joint_values = self.move_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self.move_group.set_joint_value_target(arg_list_joint_angles)
        self.move_group.plan()
        flag_plan = self.move_group.go(wait=wait)

        list_joint_values = self.move_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self.move_group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if flag_plan:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')

        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def translation_angles(self, pose, trans_x, trans_y, trans_z):
        """
        Function to compute the joint angles for the pose after translation in
        x, y, z axes
        """

        waypoints = [pose]

        # Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + trans_x
        wpose.position.y = waypoints[0].position.y + trans_y
        wpose.position.z = waypoints[0].position.z + trans_z
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5

        # Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))
        # waypoints.remove(waypoints[0])

        # Compute Cartesian Path connecting the waypoints in the list of
        # waypoints
        plan, fraction = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.1,  # Step Size
            0.0,  # Jump Threshold
            avoid_collisions=True)

        # Return the joint angles for the final pose in the plan
        return plan.joint_trajectory.points[-1].positions

    def go_to_position(self, trans_x, wait=False):
        """
        Function to go to the position on the conveyor based on the input
        offset w.r.t. to the default position
        """

        trans_x = round(trans_x, 2)

        # Make sure the values trans_x lies in the range of the keys of
        # pickup_angles
        if trans_x > max(self.pickup_angles.keys()):
            trans_x = max(self.pickup_angles.keys())
        elif trans_x < min(self.pickup_angles.keys()):
            trans_x = min(self.pickup_angles.keys())

        self.set_joint_angles(self.pickup_angles[trans_x], wait=wait)


def main():
    # Start a ROS node with name 'node_t3_ur5_1_pick_place'
    rospy.init_node('node_t3_ur5_1_pick_place', anonymous=True)

    # Create an object of the class UR5_1
    ur5 = UR5_1()
    # Create an object of the class Conveyor
    conveyor = Conveyor()
    # Create an object of the class VacuumSensor
    vac_sensor = VacuumSensor()
    # Create an object of the class LogicalCamera
    cam = LogicalCamera(conveyor, ur5)

    # Pre-compute the joint angles for all the positions on the conveyor
    ur5.calculate_pickup_angles()

    # Pick and drop the boxes in their respective bins
    done = []

    while len(done) < 3:
        while len(cam.get_packages()) == 0:
            pass

        packages = cam.get_packages()

        for package in packages:
            if package.type not in done:
                done.append(package.type)
                ur5.go_to_position(package.pose.position.z)
                while not vac_sensor.attachable or cam.overlap < 0.7:
                    pass
                ur5.move_group.stop()
                ur5.activate_vacuum_gripper()
                ur5.set_joint_angles(ur5.bin_joint_angles[len(done) - 1])
                ur5.deactivate_vacuum_gripper()

    # Stop the conveyor
    cam.kill_callback()
    # Stop the conveyor
    cam.kill_callback()
    # Move arm to default position
    ur5.set_joint_angles(ur5.joint_angles_conveyor)

    # Delete all objects
    del cam
    del conveyor
    del vac_sensor
    del ur5

    rospy.spin()


if __name__ == '__main__':
    main()
