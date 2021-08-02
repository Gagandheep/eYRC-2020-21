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
from pkg_task5.msg import PackageInfo
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import yaml
from std_msgs.msg import UInt16


def get_qr_data(arg_image):
    """Function to get QR data from image

    Args:
        arg_image (cv2 image): image to read QR code

    Returns:
        str: string stored in QR code. If no code is found, returns 'NA' 
    """

    # decodes the image to extract the qr code data
    qr_result = decode(arg_image)

    if len(qr_result) > 0:
        return qr_result[0].data
    else:
        return 'NA'


class PriorityFIFO:
    """Class for Priority based First-In First-Out queue
    """

    def __init__(self, levels=('Medicine', 'Food', 'Clothes')):
        """Constructor

        Args:
            levels (tuple, optional): priority levels in descending
                                      order. Defaults to ('Medicine',
                                      'Food', 'Clothes').
        """

        self._queue = dict()
        self.levels = levels
        self._last_operation_put = False

        for level in levels:
            self._queue[level] = list()

    def put(self, data, priority):
        """Puts data in the queue at the specified priority level

        Args:
            data (any): data to be added to queue
            priority (any non mutable): priority of the data, value
                                        should be from levels
        """

        self._queue[priority].append(data)
        self._last_operation_put = True

    def get(self):
        """Returns the highest priority data from the queue

        Returns:
            any: highest priority data from the queue
        """

        for priority, item in enumerate(self.levels):
            if len(self._queue[item]) != 0:
                return self._queue[item][0], priority

        else:
            return None

    def remove_first(self, priority):
        """Removes the data of the priority level from the queue

        Args:
            priority (any non mutable): priority of the data, value
                                        should be from levels
        """

        self._queue[priority].pop(0)
        self._last_operation_put = False

    @property
    def length(self):
        """
        Returns:
            int: length of the queue
        """

        return sum(len(i) for i in self._queue.values())

    @property
    def last_operation_put(self):
        """
        Returns:
            bool: True if last operation performed on the queue was put
        """

        return self._last_operation_put


class PackageInfoPublisher:
    """Class for publishing package information
    """

    def __init__(self, topic, queue_size=3):
        """Constructor

        Args:
            topic (str): topic to publish on
            queue_size (int, optional): queue size of the publisher.
                                        Defaults to 3.
        """

        self.topic = topic
        self.handle = rospy.Publisher(topic, PackageInfo,
                                      queue_size=queue_size)

    def publish(self, color, offset, order_id):
        """Publishes package info

        Args:
            color (int): number corresponding to the color of the
                         package
            offset (float): offset of the package on the conveyor
                            relative to the camera
            order_id (int): order ID of the package
        """

        msg = PackageInfo()
        msg.color = color
        msg.offset = offset
        msg.order_id = order_id
        self.handle.publish(msg)


class ShippingIDPublisher:
    """Class for publishing Shipping ID
    """

    def __init__(self, topic, queue_size=3):
        """Constructor

        Args:
            topic (str): topic to publish on
            queue_size (int, optional): queue size of the publisher.
                                        Defaults to 3.
        """

        self.topic = topic
        self.handle = rospy.Publisher(topic, UInt16, queue_size=queue_size)

    def publish(self, order_id):
        """Publishes Shipping ID

        Args:
            order_id (int): order ID of the package
        """

        msg = UInt16()
        msg.data = order_id
        self.handle.publish(msg)


class Camera2D:
    """Class for 2D Camera
    """

    def __init__(self):
        """Constructor
        """

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,
                                          self.callback)

        self.image = None

    def callback(self, data):
        """Callback function

        Args:
            data (ros msg): callback data
        """

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
        """Returns a list containing the color of the packages

        Returns:
            list(str): list containing the color of the packages, 'NA'
                       if no package
        """

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
        """Called automatically when the object is deleted
        """

        cv2.destroyAllWindows()


class Conveyor:
    """Class to control the Conveyor
    """

    def __init__(self):
        """Constructor
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
        Returns:
            bool: returns True if current power is more than 0, else False
        """

        return self._is_moving

    def move(self, power=100):
        """Sets the conveyor speed

        Args:
            power (int, optional): sets the conveyor speed based on
                                   power. Defaults to 100 (max).
        """

        # Perform check to avoid setting same power repeatedly which may lead
        # to crashes
        if power != self._current_power:
            self._current_power = power
            self._is_moving = power != 0
            req = conveyorBeltPowerMsgRequest(power=power)
            self.conveyor_service(req)

    def stop(self):
        """Stops the conveyor
        """

        self.move(power=0)


class VacuumSensor:
    """Class to read the vacuum sensor
    """

    def __init__(self, topic):
        """Constructor

        Args:
            topic (str): topic to subscribe to
        """

        rospy.Subscriber(topic,
                         LogicalCameraImage,
                         self.attachable_callback)

        self._attachable = False

    def attachable_callback(self, data):
        """Subscriber callback function

        Args:
            data (ros msg): callback data
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
        Returns:
            bool: True if a grippable object is detected, else False
        """

        return self._attachable


class LogicalCamera:
    """Class to read the Logical Camera
    """

    def __init__(self, topic, ur5, callback=None, *args):
        """Constructor

        Args:
            topic (str): topic to subscribe to
            ur5 (UR5): UR5 arm to get overlap data
            callback (function, optional): callback function. Defaults
                                           to None.
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
        """Subscriber callback function

        Args:
            data (ros msg): callback variable
        """

        self._models = data.models
        if self.callback is not None:
            self.callback(self, *self.callback_args)

    def update_overlap(self, box):
        """Function to calculate the overlap between the positions of
        the package and the vacuum gripper

        Args:
            box (Pose): pose of the box with which overlap has to be
                        calculated
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
        """Returns a list of models in the FOV of the camera

        Returns:
            list: list of all models
        """

        return self._models

    @property
    def overlap(self):
        """
        Returns:
            float: overlap value ranging between 0 and 1 inclusive
        """

        return self._overlap

    def get_packages(self):
        """
        Returns:
            list: list of packages
        """

        packages = []
        if len(self._models) > 0:
            for model in self._models:
                if model.type[:4] == 'pack':
                    packages.append(model)

        return packages

    def get_types(self):
        """Returns a list types of the packages in the FOV of the camera

        Returns:
            list(str): list of types of the models
        """

        types = []
        if len(self._models) > 0:
            for model in self._models:
                if model.type != 'ur5':
                    types.append(model.type)

        return types


class UR5:
    """Class to control the UR5 arm
    """

    def __init__(self, robot_ns):
        """Constructor

        Args:
            robot_ns (str): namespace of the robot
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
        self.pkg_path = rp.get_path('pkg_task5')

    @property
    def attached(self):
        """
        Returns:
            bool: returns True if vacuum gripper is attached to an
                  object
        """

        return self._attached

    def activate_vacuum_gripper(self):
        """Activates the vacuum gripper
        """

        # Call ROS service to activate the vacuum gripper
        req = vacuumGripperRequest(activate_vacuum_gripper=True)
        self.vacuum_service(req)

        self._attached = True

    def deactivate_vacuum_gripper(self):
        """Deactivates the vacuum gripper
        """

        # Call ROS service to deactivate the vacuum gripper
        req = vacuumGripperRequest(activate_vacuum_gripper=False)
        self.vacuum_service(req)

        self._attached = False

    def calculate_pickup_angles(self, step=0.01,
                                longitudinal_range=(-0.1, 0.2)):
        """Function to calculate and store the joint angles for all the
        positions on the conveyor along the z-axis of the logical camera

        Args:
            step (float, optional): precision of the steps. Defaults to
                                    0.01.
            longitudinal_range (tuple, optional): range of the z values.
                                                Defaults to (-0.1, 0.2).
        """

        self.pickup_angles = dict()

        i = longitudinal_range[0]
        # Iterates over the range of values from -0.1 to 0.2 and
        # calculates the joint angles.
        while i <= longitudinal_range[1]:
            self.pickup_angles[i] = self.translation_angles(
                self.move_group.get_current_pose().pose,
                i, 0, 0)
            i = round(i + step, 2)

    def set_joint_angles(self, arg_list_joint_angles, mode=0, wait=True):
        """Sets the joint angles of the UR5 arm to the given values

        Args:
            arg_list_joint_angles (list(float)): list of joint angles
            mode (int, optional): mode of operation
                                  (0: Plan and Execute;
                                   1:Try Execution, if failed, Plan and
                                     Execute;
                                   2: Plan only).
                                  Defaults to 0.
            wait (bool, optional): wait until execution is completed.
                                   Defaults to True.

        Returns:
            bool: True if successful, else False
        """

        list_joint_values = self.move_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self.move_group.set_joint_value_target(arg_list_joint_angles)
        flag_plan = False

        # Plan and execute mode
        if mode == 0:
            while not flag_plan:
                self.computed_plan = self.move_group.plan()
                flag_plan = self.move_group.execute(self.computed_plan,
                                                    wait=wait)

        # Execution Mode if failed falss back to Plan and Execute Mode
        elif mode == 1:
            flag_plan = self.move_group.execute(self.computed_plan, wait=wait)
            if not flag_plan:
                return self.set_joint_angles(arg_list_joint_angles)

        # Plan only Mode
        elif mode == 2:
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
        """Function to compute the joint angles for the pose after
        translation in x, y, z axes

        Args:
            pose (Pose): current pose of the arm
            trans_x (float): translation along x axis
            trans_y (float): translation along y axis
            trans_z (float): translation along z axis

        Returns:
            list(float): joint angles after translation
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

    def go_to_position(self, offset, wait=True):
        """Function to go to the position on the conveyor based on the
        input offset w.r.t. to the default position

        Args:
            offset (float): offset to be applied
            wait (bool, optional): wait until execution is completed.
            Defaults to True.
        """

        offset = round(offset, 2)

        # Make sure the values offset lies in the range of the keys of
        # pickup_angles
        if offset > max(self.pickup_angles.keys()):
            offset = max(self.pickup_angles.keys())
        elif offset < min(self.pickup_angles.keys()):
            offset = min(self.pickup_angles.keys())

        self.set_joint_angles(self.pickup_angles[offset], wait=wait)

    def play_trajectory(self, name):
        """Reads a saved plan and executes it

        Args:
            name (str): name of the file containing the plan

        Returns:
            bool: True if execution was successful, else False
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
    """Class for storing package data
    """

    def __init__(self, color, x, y):
        """Constructor

        Args:
            color (str): color of the package
            x (int): row index of the package
            y (int): column index of the package
        """

        self.color_dict = {'red': 0, 'yellow': 1, 'green': 2}
        self._position = (x, y)
        self._color = color
        self._type = ['Medicine', 'Food', 'Clothes'][self.color_dict[color]]

    @property
    def position(self):
        """
        Returns:
            tuple: row and column indices of the package
        """
        return self._position

    @property
    def type(self):
        """
        Returns:
            str: type of the package
        """
        return self._type

    @property
    def color(self):
        """
        Returns:
            str: color of the package
        """
        return self._color

    @property
    def color_id(self):
        """
        Returns:
            int: color ID of the package
                 ('red': 0, 'yellow': 1, 'green': 2)
        """
        return self.color_dict[self._color]


class Shelf:
    """Class for storing info about the items on the shelf
    """

    def __init__(self, colors):
        """Constructor

        Args:
            colors (list(str)): list of colors of the package starting
                                from top right to bottom left going
                                sideways
        """

        # Intializes a 2D list of colors of the package starting from
        # the top right to bottom left going sideways
        self.slots = [[None for _ in range(3)] for _ in range(4)]

        for i in range(12):
            if colors[i] == 'NA':
                continue

            x = i // 3
            y = i % 3
            self.slots[x][y] = Package(colors[i], x, y)

    def get(self, *position):
        """gets the ith package or the package on the xth row and yth
        column on the shelf

        Returns:
            Package: Package object for the package at the specified
                     position
        """
        # gets the ith package or the package on the xth row and yth
        # column on the shelf
        if len(position) == 1:
            x = position[0] // 3
            y = position[0] % 3
            return self.slots[x][y]
        else:
            return self.slots[position[0]][position[1]]

    def remove(self, package):
        """Removes package from the shelf

        Args:
            package (Package): package to be removed
        """

        x, y = package.position[0], package.position[1]
        self.slots[x][y] = None
        del package  # removes the pacage from the shelf

    def find(self, package_type):
        """Finds a package of the specified type on the shelf

        Args:
            package_type (str): type of the package

        Returns:
            tuple(int): row and column indices of the object on the
                        shelf, (-1, -1) if not found
        """
        # iterates over the range of number of packages and finds the
        # package of specific type on the shelf.
        for i in range(12):
            pkg = self.get(i)
            if pkg is not None and pkg.type == package_type:
                return pkg.position

        return -1, -1

    def find_all_indices(self, package_type):
        """Finds the indices of all the packages of the specified type

        Args:
            package_type (str): type of the package

        Returns:
            list(int): indices of the packages on the shelf
        """
        # iterates over the range of number of packages and finds the
        # indices of all the packages of the specified type.
        indices = list()
        for i in range(12):
            pkg = self.get(i)
            if pkg is not None and pkg.type == package_type:
                indices.append(pkg.position[0] * 3 + pkg.position[1])

        return indices


class TrajectoryPlans:
    """Class for managing the saved trajectory plans
    """

    def __init__(self, costs):
        """Constructor

        Args:
            costs (dict((int, int) -> float)): dictionary containing
                                               time taken for the
                                               execution of plan from
                                               one state to another
        """
        self.costs = costs

    def get_min_cost_plan(self, current_state, target_states):
        """Returns the target state with minimum execution time

        Args:
            current_state (int): current state of the arm
            target_states (list(int)): list of target states

        Returns:
            int: target state with minimum execution time
        """
        # accepts the current state and target states and calculates the
        # target state with minimum execution time.
        min_cost = 9999999999
        ret_state = -1

        for target_state in target_states:
            cost = self.costs[
                       (current_state, target_state)]
            if cost < min_cost:
                min_cost = cost
                ret_state = target_state

        return ret_state

    def get_optimal_cost_plan(self, current_state, target_states, shelf, item,
                              queue_length):
        """Returns the optimal target state based on execution time and
        next possible states

        Args:
            current_state (int): current state of the arm
            target_states (list(int)): list of target states
            shelf (Shelf): shelf object
            item (str): type of item at the target states
            queue_length (int): length of the PriorityQueue object

        Returns:
            int: optimal target state
        """
        # accepts the current state and target state and calculates the
        # optimal cost plan.
        min_cost = 9999999999
        ret_state = -1

        if item == 'Medicine':
            return self.get_min_cost_plan(current_state, target_states)

        elif item == 'Food':
            next_states = shelf.find_all_indices('Medicine')

            for target_state in target_states:
                cost = self.costs[(current_state, target_state)] * (
                        queue_length > 1) + self.costs[(target_state,
                                                        self.get_min_cost_plan(
                                                            target_state,
                                                            next_states))]

                if cost < min_cost:
                    min_cost = cost
                    ret_state = target_state

        else:
            next_states_med = shelf.find_all_indices('Medicine')
            next_states_fd = shelf.find_all_indices('Food')
            for target_state in target_states:
                cost_med = self.costs[(current_state, target_state)] * (
                        queue_length > 1) + self.costs[(target_state,
                                                        self.get_min_cost_plan(
                                                            target_state,
                                                            next_states_med))]
                cost_fd = self.costs[(current_state, target_state)] * (
                        queue_length > 1) + self.costs[(target_state,
                                                        self.get_min_cost_plan(
                                                            target_state,
                                                            next_states_fd))]
                cost = (cost_med + cost_fd) / 2.0
                if cost < min_cost:
                    min_cost = cost
                    ret_state = target_state

        return ret_state
