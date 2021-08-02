#!/usr/bin/env python


import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest


class UR5_1:

    def __init__(self):
        """
        class init
        """
        self.commander = moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.box_name = 'package$1'
        self.move_group = moveit_commander.MoveGroupCommander(
            'ur5_1_planning_group')
        self.robot = moveit_commander.RobotCommander()
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1)

        self.exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory',
            moveit_msgs.msg.ExecuteTrajectoryAction)
        self.exectute_trajectory_client.wait_for_server()
        self.ee_link = self.move_group.get_end_effector_link()

        vacuum_gripper_service = '/eyrc/vb/ur5_1/activate_vacuum_gripper'

        rospy.wait_for_service(vacuum_gripper_service)
        self.vacuum_service = rospy.ServiceProxy(vacuum_gripper_service,
                                                 vacuumGripper)

    def add_box(self, timeout=4):
        """
        Function to add a box in the RViz scene
        """

        # Side length of the box
        box_size = 0.16

        # Set pose of the box
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'world'
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.45
        box_pose.pose.position.z = 1.92

        # Add box to scene
        self.scene.add_box(self.box_name, box_pose, size=(box_size,
                                                          box_size,
                                                          box_size))

        # Wait for update and return status
        return self.wait_for_state_update(box_is_known=True,
                                          timeout=timeout)

    def attach_box(self, timeout=4):
        """
        Function to attach the box to the EE of UR5
        """

        group = 'ur5_1_planning_group'
        touch_links = self.robot.get_link_names(group=group)

        # Attach box to EE
        self.scene.attach_box(self.ee_link,
                              self.box_name,
                              touch_links=touch_links)

        # Wait for update and return status
        return self.wait_for_state_update(box_is_attached=True,
                                          box_is_known=False,
                                          timeout=timeout)

    def detach_box(self, timeout=4):
        """
        Function to detach the box from the EE of UR5
        """

        # Detach box from EE
        self.scene.remove_attached_object(self.ee_link, name=self.box_name)

        # Wait for update and return status
        return self.wait_for_state_update(box_is_known=True,
                                          box_is_attached=False,
                                          timeout=timeout)

    def remove_box(self, timeout=4):
        """
        Function to remove the box from the RViz scene
        """

        # Remove box from scene
        self.scene.remove_world_object(self.box_name)

        # Wait for update and return status
        return self.wait_for_state_update(box_is_attached=False,
                                          box_is_known=False,
                                          timeout=timeout)

    def activate_vacuum_gripper(self):
        """
        Function to activate the vacuum gripper
        """

        # Call ROS service to activate the vacuum gripper
        req = vacuumGripperRequest(activate_vacuum_gripper=True)
        self.vacuum_service(req)

    def deactivate_vacuum_gripper(self):
        """
        Function to deactivate the vacuum gripper
        """

        # Call ROS service to deactivate the vacuum gripper
        req = vacuumGripperRequest(activate_vacuum_gripper=False)
        self.vacuum_service(req)

    def wait_for_state_update(self,
                              box_is_known=False,
                              box_is_attached=False,
                              timeout=4):
        """
        Function to wait for state update
        """

        start = rospy.get_time()
        seconds = rospy.get_time()
        while seconds - start < timeout and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if box_is_attached == is_attached and box_is_known == is_known:
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def go_to_pose(self, arg_pose, attempts=25):
        """
        Function to make the UR5 EE to go to the specified pose
        """

        # Log initial pose
        pose_values = self.move_group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        # Set target pose
        self.move_group.set_pose_target(arg_pose)

        # Try multiple times to got to the target pose
        for _ in range(attempts):
            # Go to target pose (wait=False for Async Move)
            flag_plan = self.move_group.go(wait=True)

            # Break from loop if going to target pose was successful
            if flag_plan:
                break

        # Log final pose
        pose_values = self.move_group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        # Log the joint angles
        list_joint_values = self.move_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if flag_plan:
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' +
                ">>> go_to_pose() Failed. Solution for Pose not Found."
                + '\033[0m')

        return flag_plan


def main():

    # Start a ROS node with name 'node_t2_ur5_1_pick_place'
    rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

    # Create an object of the class UR5_1
    ur5 = UR5_1()

    # Pose 1: Pose for moving the EE near to the box in the shelf
    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = 0.0
    ur5_pose_1.position.y = 0.25
    ur5_pose_1.position.z = 1.91
    ur5_pose_1.orientation.x = 0.0
    ur5_pose_1.orientation.y = 0.0
    ur5_pose_1.orientation.z = 0.0
    ur5_pose_1.orientation.w = 0.0

    # Pose 2: Pose for moving the EE to a position above the bin
    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = -0.817261772949
    ur5_pose_2.position.y = -0.109110076352
    ur5_pose_2.position.z = 0.94446979642
    ur5_pose_2.orientation.x = -1
    ur5_pose_2.orientation.y = 0
    ur5_pose_2.orientation.z = 0
    ur5_pose_2.orientation.w = 1

    # Add box in RViz scene
    ur5.add_box()

    # Go to pose 1
    ur5.go_to_pose(ur5_pose_1)

    # Call the ROS service to activate the vacuum gripper
    ur5.activate_vacuum_gripper()

    # Attach box in RViz scene to the UR5_1 EE
    ur5.attach_box()

    # Go to pose 2
    ur5.go_to_pose(ur5_pose_2)

    # Call the ROS service to deactivate the vacuum gripper
    ur5.deactivate_vacuum_gripper()

    # detach box in RViz scene from the UR5_1 EE
    ur5.detach_box()

    # Remove the box from the RViz scene
    ur5.remove_box()

    # delete the ur5 object
    del ur5

    rospy.spin()


if __name__ == '__main__':
    main()
