#!/usr/bin/env python

# Imports
from warehouse.warehouse import *

from pkg_ros_iot_bridge.msg import msgRosIotAction  # used by ROS Actions
from pkg_ros_iot_bridge.msg import msgRosIotGoal  # Goal Messages
from pkg_ros_iot_bridge.msg import msgMqttSub  # MQTT Subscription Messages

import json
from datetime import datetime
from datetime import timedelta


def cam_callback(obj, publisher):
    """Camera Callback function to publish color and offset data

    Args:
        obj (LogicalCamera): LogicalCamera object
        publisher (PackageInfoPublisher): Publisher class to be used
    """

    global color_id  # zero corresponds to red, one to yellow, two to green.

    # Publish color id when the package position along z axis is less
    # than 1.2
    for package in obj.get_packages():
        # check if package position is less than 1.2
        if color_id is not None and package.pose.position.z < 1.2:
            # publish color ID if the conditions are true
            publisher.publish(color_id,
                              package.pose.position.z,
                              curr_id)
            color_id = None


def shipping_id_subscriber_callback(data, ac_ros_iot):
    """Shipping ID Callback function to publish data about the order
    shipped on the sheet 'OrdersShipped'

    Args:
        data (ros msg): callback data
        ac_ros_iot (SimpleActionClient): Publisher class to be used
    """

    global order_data  # stores incoming orders

    order_id = data.data  # id of the order
    data_dict = order_data[order_id]  # details of the order
    date_time = datetime.now()  # get the current time
    json_data_dict = {
        'id': 'OrdersShipped',  # Sheet name
        'Team Id': 'VB#0042',
        'Unique Id': 'GrRbByYg',
        'Order ID': order_id,  # Id of the order
        'City': data_dict['City'],  # Shipping destination
        'Item': data_dict['Item'],  # Item to be shipped
        'Priority': data_dict['Priority'],  # Priority of the order
        'Shipped Quantity': data_dict['Dispatch Quantity'],
        # Quantity of the order
        'Cost': data_dict['Cost'],  # Cost of the order
        'Shipped Status': 'YES',  # Status of the order
        'Shipped Date and Time': str(date_time)[:-7],
        # Date and time the order was shipped
        'Estimated Time of Delivery': str(date_time + timedelta(
            days={'HP': 1, 'MP': 3, 'LP': 5}[data_dict['Priority']]))[:10],
        # Estimated time to deliver the order
        'email_1': emails[0],
        'notify_1': number == str(order_id),
        'email_2': emails[1],
        'notify_2': notify[1]
    }
    json_data = json.dumps(
        json_data_dict)  # Converts the dictionary to string format.

    goal_ros_iot = msgRosIotGoal('http',
                                 'sheet',
                                 "eyrc/GrRbByYg/ros_to_iot",
                                 json_data)  # Create a ROS request.
    ac_ros_iot.send_goal(goal_ros_iot)  # Send request to ROS IOT Bridge


def mqtt_subscriber_callback(data, args):
    """Callback function executed when a message is received

    Args:
       data (ros msg): callback data
       args (List): List of parameters used in function
    """

    global count                                                        ####
    global number                                                       ####

    ac_ros_iot, queue = args
    data_dict = json.loads(data.message)
    json_data_dict = {
        'id': 'IncomingOrders',
        'Team Id': 'VB#0042',
        'Unique Id': 'GrRbByYg',
        'Order ID': data_dict['order_id'],
        'Order Date and Time': data_dict['order_time'],
        'Item': data_dict['item'],
        'Priority': {'Medicine': 'HP', 'Food': 'MP', 'Clothes': 'LP'}[
            data_dict['item']],
        'Order Quantity': data_dict['qty'],
        'City': data_dict['city'],
        'Longitude': data_dict['lon'],
        'Latitude': data_dict['lat'],
        'Cost': {'Medicine': 450, 'Food': 250, 'Clothes': 150}[
            data_dict['item']]
    }
    json_data = json.dumps(json_data_dict)
    queue.put(json_data_dict.copy(),
              data_dict['item'])  # Add order data to queue.

    count += 1                                                          ####
    if count == 9:                                                      ####
        number = data_dict['order_id']                                  ####

    goal_ros_iot = msgRosIotGoal('http',
                                 'sheet',
                                 'eyrc/GrRbByYg/ros_to_iot',
                                 json_data)
    ac_ros_iot.send_goal(goal_ros_iot)  # Publish data to sheet.


def main():
    """Main Function
    """

    global color_id
    global curr_id
    global order_data

    # Start a ROS node with name 'node_t3_ur5_1_pick_place'
    rospy.init_node('node_t3_ur5_1', anonymous=True)
    ac_ros_iot = actionlib.SimpleActionClient('/action_ros_iot',
                                              msgRosIotAction)
    ac_ros_iot.wait_for_server()
    sub_data = msgRosIotGoal('mqtt', 'sub', '/eyrc/vb/GrRbByYg/orders',
                             '')  # Subscribe to ROS topic GrRbByYg.
    ac_ros_iot.send_goal(sub_data)  # Publish data to sheet.

    # Create an object of the class UR5_1
    ur5_1 = UR5('ur5_1')
    # Create an object of the class Camera2D
    cam2d = Camera2D()
    # Start a publisher to publish package data
    package_info_publisher = PackageInfoPublisher('/package_data')
    # Create an object of the class LogicalCamera
    cam_logical = LogicalCamera('/eyrc/vb/logical_camera_1', ur5_1,
                                cam_callback, package_info_publisher)

    queue = PriorityFIFO()
    costs = dict()
    # File path
    file_path = ur5_1.pkg_path + '/config/saved_trajectories/costs.txt'

    # Load file from path
    with open(file_path, 'r') as file_open:
        for line in file_open.readlines():
            current, target, cost = list(map(int, line.rstrip().split()))
            costs[(current, target)] = cost

    plans = TrajectoryPlans(costs)

    rate = rospy.Rate(500)

    rospy.sleep(5)

    # Get colors of packages from shelf
    colors = cam2d.get_colors()
    # Create an object of the class Shelf
    shelf = Shelf(colors)

    # Subscribe to topic '/ros_iot_bridge/mqtt/sub'
    subs_mqtt = rospy.Subscriber('/ros_iot_bridge/mqtt/sub',
                                 msgMqttSub,
                                 mqtt_subscriber_callback,
                                 [ac_ros_iot,
                                  queue])

    # Subscribe to topic '/shipping_id'
    subs_id = rospy.Subscriber('/shipping_id',
                               UInt16,
                               shipping_id_subscriber_callback,
                               ac_ros_iot)  # Subscriber ID.

    # Go to default pose
    ur5_1.play_trajectory('ur5_1_default')
    ur5_1.move_group.stop()
    ur5_state = -1

    for i in range(12):
        pkg = shelf.get(i)  # Get the ith package.
        date_time = datetime.now()
        # Store the details of the package in the dictionary.
        json_data_dict = {
            'id': 'Inventory',
            'Team Id': 'VB#0042',
            'Unique Id': 'GrRbByYg',
            'SKU': pkg.color[0].upper() + str(pkg.position[0]) + str(
                pkg.position[1]) + ('0' + str(date_time.month))[-2:] + str(
                date_time.year)[-2:],
            'Item': pkg.type,
            'Priority': {0: 'HP', 1: 'MP', 2: 'LP'}[pkg.color_id],
            'Storage Number': 'R' + str(pkg.position[0]) + ' C' + str(
                pkg.position[1]),
            'Cost': {0: 450, 1: 250, 2: 150}[pkg.color_id],
            'Quantity': 1
        }

        # Convert the dictionary to string format
        json_data = json.dumps(json_data_dict)

        goal_ros_iot = msgRosIotGoal('http',
                                     'sheet',
                                     "eyrc/GrRbByYg/ros_to_iot",
                                     json_data)
        ac_ros_iot.send_goal(goal_ros_iot)  # Publish data to sheet
        rospy.sleep(1)

    while True:
        while queue.length == 0:
            rate.sleep()

        # Get order information and priority of the order
        order_info, priority = queue.get()
        # Find the indices in which the item is available
        indices = shelf.find_all_indices(order_info['Item'])

        ur5_state = plans.get_optimal_cost_plan(ur5_state, indices, shelf,
                                                order_info['Item'],
                                                queue.length)

        # Get the package from the shelf based on the plan
        package = shelf.get(ur5_state)
        x, y = package.position

        # Move EE to Package
        while not ur5_1.play_trajectory('ur5_1_default_{}{}'.format(x, y)):
            pass
        ur5_1.move_group.stop()

        while True:
            # Check for new order
            new_order_info, new_priority = queue.get()
            # Check if the new order has higher priority than current
            if new_priority < priority:
                # Process the new order if its of higher priority
                # Repeat the same process to select the required package
                indices = shelf.find_all_indices(new_order_info['Item'])
                ur5_state = plans.get_min_cost_plan(ur5_state, indices)
                package = shelf.get(ur5_state)
                x_, y_ = package.position

                # Move EE to Package
                while not ur5_1.play_trajectory(
                        'ur5_1_{}{}_{}{}'.format(x, y, x_, y_)):
                    pass
                ur5_1.move_group.stop()

                x, y = x_, y_
                # Update the order info and priority to that of the
                # latest order
                order_info = new_order_info
                priority = new_priority

            else:
                queue.remove_first(order_info['Item'])
                break

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
        curr_id = int(order_info['Order ID'])

        # Publish to OrdersDispatched
        date_time = datetime.now()
        json_data_dict = {
            'id': 'OrdersDispatched',
            'Team Id': 'VB#0042',
            'Unique Id': 'GrRbByYg',
            'Order ID': order_info['Order ID'],
            'City': order_info['City'],
            'Item': order_info['Item'],
            'Priority': order_info['Priority'],
            'Dispatch Quantity': order_info['Order Quantity'],
            'Cost': order_info['Cost'],
            'Dispatch Status': 'YES',
            'Dispatch Date and Time': str(date_time)[:-7],
            'email_1': emails[0],
            'notify_1': order_info['Order ID'] == number,
            'email_2': emails[1],
            'notify_2': notify[1]
        }
        json_data = json.dumps(json_data_dict)

        goal_ros_iot = msgRosIotGoal('http',
                                     'sheet',
                                     "eyrc/GrRbByYg/ros_to_iot",
                                     json_data)
        ac_ros_iot.send_goal(goal_ros_iot)  # Publish data to sheet

        order_data[int(json_data_dict['Order ID'])] = json_data_dict.copy()

        shelf.remove(package)  # Remove the package from the shelf


if __name__ == '__main__':
    color_id = None  # bin number the box to be put in
    curr_id = None  # current id
    order_data = dict()  # order data received from node_t5_ur5_1

    # Email addresses for notifications (MAX: 2)
    emails = ['eyrc.vb.0042@gmail.com', 'eyrc.vb.0000@gmail.com']
    # Set True to send email for every eamil address in 'emails' list
    notify = [True, True]

    count = 0
    number = 0

    # Call main function
    main()
