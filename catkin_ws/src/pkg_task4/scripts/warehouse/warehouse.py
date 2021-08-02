import math
import sys
import rospy
import copy
import cv2
from pyzbar.pyzbar import decode
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.srv import vacuumGripper, vacuumGripperRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsg, conveyorBeltPowerMsgRequest
from pkg_vb_sim.msg import LogicalCameraImage
from sensor_msgs.msg import Image
from pkg_task4.msg import PackageInfo
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import yaml


def get_qr_data(arg_image):
    qr_result = decode(arg_image)

    if len(qr_result) > 0:
        return qr_result[0].data
    else:
        return 'NA'


class PackageInfoPublisher:

    def __init__(self, topic, queue_size=3):
        self.topic = topic
        self.handle = rospy.Publisher(topic, PackageInfo,
                                      queue_size=queue_size)

    def publish(self, color, offset):
        msg = PackageInfo()
        msg.color = color
        msg.offset = offset
        self.handle.publish(msg)


class Camera2D:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,
                                          self.callback)

        self.image = None

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image = cv_image
        except CvBridgeError as e:
            cv_image = self.image
            rospy.logerr(e)

        # Resize a 720x1280 image to 360x640 to fit it on the screen
        resized_image = cv2.resize(cv_image, (360, 640))

        cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
        cv2.waitKey(3)

    def get_colors(self):
        colors = list()
        row_start = [300, 480, 620, 780]
        width = 180
        height = 130

        for row in range(4):
            for col in range(3):
                sub_image = self.image[row_start[row]:row_start[row] + height,
                            90 + col * width:90 + col * width + 180]
                colors.append(get_qr_data(sub_image))

        return colors

    def __del__(self):
        cv2.destroyAllWindows()


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

    def __init__(self, topic):
        """
        class init
        """

        rospy.Subscriber(topic,
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

    def __init__(self, topic, ur5, callback=None, *args):
        """
        class init
        """

        self._models = []
        self._overlap = 0.0

        self.subscriber = rospy.Subscriber(topic,
                                           LogicalCameraImage,
                                           self.subscriber_callback)

        self.ur5 = ur5
        self.move_group = ur5.move_group
        self.callback = callback
        self.callback_args = args

    def subscriber_callback(self, data):
        """
        Callback function to update models
        """

        self._models = data.models
        if self.callback is not None:
            self.callback(self, *self.callback_args)

    def update_overlap(self, box):
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
        if len(self._models) > 0:
            for model in self._models:
                if model.type[:4] == 'pack':
                    packages.append(model)

        return packages

    def get_types(self):
        """
        Function to return the names of the packages in the FOV of the camera
        """

        types = []
        if len(self._models) > 0:
            for model in self._models:
                if model.type != 'ur5':
                    types.append(model.type)

        return types


class UR5:

    def __init__(self, robot_ns):
        """
        class init
        """

        self._robot_ns = '/' + robot_ns
        self.commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(
            robot_description=self._robot_ns + "/robot_description",
            ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(
            ns=self._robot_ns)
        self.move_group = moveit_commander.MoveGroupCommander(
            'manipulator',
            robot_description=self._robot_ns + "/robot_description",
            ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(
            self._robot_ns + '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            self._robot_ns + '/execute_trajectory',
            moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        vacuum_service_name = '/eyrc/vb/ur5/activate_vacuum_gripper' \
                              + self._robot_ns

        rospy.wait_for_service(vacuum_service_name)
        self.vacuum_service = rospy.ServiceProxy(vacuum_service_name,
                                                 vacuumGripper)

        self.ee_link = self.move_group.get_end_effector_link()

        self._attached = False
        self.computed_plan = None
        self.pickup_angles = dict()
        rp = rospkg.RosPack()
        self.pkg_path = rp.get_path('pkg_task4')

    @property
    def attached(self):
        """
        Class property
        Returns attached state vacuum gripper
        """

        return self._attached

    def wait_for_state_update(self,
                              box,
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
            attached_objects = self._scene.get_attached_objects([box])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box in self._scene.get_known_object_names()

            # Test if we are in the expected state
            if box_is_attached == is_attached and box_is_known == is_known:
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

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
            self.pickup_angles[i] = self.translation_angles(
                self.move_group.get_current_pose().pose,
                i, 0, 0)
            i = round(i + step, 2)

    def set_joint_angles(self, arg_list_joint_angles, mode=0, wait=True):
        """
        Function to set the joint angles of the UR5 arm
        """

        list_joint_values = self.move_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self.move_group.set_joint_value_target(arg_list_joint_angles)
        flag_plan = False

        if mode == 0:
            while not flag_plan:
                self.computed_plan = self.move_group.plan()
                flag_plan = self.move_group.execute(self.computed_plan,
                                                    wait=wait)

        elif mode == 1:
            flag_plan = self.move_group.execute(self.computed_plan, wait=wait)
            if not flag_plan:
                return self.set_joint_angles(arg_list_joint_angles)

        else:
            self.computed_plan = self.move_group.plan()
            return True

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

        waypoints.remove(waypoints[0])

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

    def go_to_position(self, trans_x, wait=True):
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

    def play_trajectory(self, name):
        """
        Function to set the joint angles of the UR5 arm
        """

        # File path
        file_path = self.pkg_path + '/config/saved_trajectories' \
                    + self._robot_ns + '/' + name + '.yaml'

        # Load file from path
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        # Execute path
        return self.move_group.execute(loaded_plan)


class Package:

    def __init__(self, color, x, y):
        """
        class init
        """

        self.color_dict = {'red': 0, 'yellow': 1, 'green': 2}
        self._position = (x, y)
        self._color = color

    @property
    def position(self):
        """
        Class property
        Returns the position of the package in the shelf
        """
        return self._position

    @property
    def color(self):
        """
        Class property
        Returns the color of the package
        """
        return self._color

    @property
    def color_id(self):
        """
        Class property
        Returns the target bin number for the package
        """
        return self.color_dict[self._color]

    @property
    def name(self):
        """
        Class property
        Returns the name of the package
        """
        return 'packagen' + str(self._position[0]) + str(self._position[1])


class Shelf:

    def __init__(self, colors):
        """
        class init
        """

        self.slots = [[None for _ in range(3)] for _ in range(4)]

        for i in range(12):
            if colors[i] == 'NA':
                continue

            x = i // 3
            y = i % 3
            self.slots[x][y] = Package(colors[i], x, y)

    def get(self, *position):
        """
        Function to get the ith package in the shelf or the package in the xth row and yth column
        x, y, z axes
        """

        if len(position) == 1:
            x = position[0] // 3
            y = position[0] % 3
            return self.slots[x][y]
        else:
            return self.slots[position[0]][position[1]]

    def remove(self, package):
        """
        Function to remove a package from the Shelf
        """

        x, y = package.position[0], package.position[1]
        del self.slots[x][y]
        self.slots[x][y] = None
