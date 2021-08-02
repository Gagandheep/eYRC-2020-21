#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import rospy
import actionlib
import threading
import json

# Message Class imports
from pkg_ros_iot_bridge.msg import msgRosIotAction  # used by ROS Actions
from pkg_ros_iot_bridge.msg import msgRosIotGoal  # Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult  # Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback  # Feedback Messages
from pkg_ros_iot_bridge.msg import msgMqttSub  # MQTT Subscription Messages

from pyiot import iot  # Custom Python Module to perform MQTT Tasks


class ActionServerRosIotBridge:
    """Class for Action Server for communication over MQTT and HTTP
    """

    def __init__(self):
        """Constructor
        """

        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        
        # * self.on_goal - It is the function pointer which points to a 
        # function which will be called when the Action Server receives a 
        # Goal.

        # * self.on_cancel - It is the function pointer which points to a 
        # function which will be called when the Action Server receives a 
        # Cancel Request.
        

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = \
            param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_google_apps_spread_sheet_id = \
            param_config_iot['google_apps']['spread_sheet_id']
        self._config_google_apps_spread_sheet_id_eyrc = \
            param_config_iot['google_apps']['spread_sheet_id_eyrc']
        print(param_config_iot)

        print('\n\n\n')
        print('To pass data to ROS IoT Bridge node, MQTT client should '
              'publish to this MQTT Topic:', self._config_mqtt_sub_topic)
        print('To get data from ROS IoT Bridge node, MQTT client should '
              'publish to this MQTT Topic:', self._config_mqtt_pub_topic)
        print('\n\n\n')

        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a
        # ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub)
        # to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(
            self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)

        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in
        # 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a
        # message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    def mqtt_sub_callback(self, client, userdata, message):
        """Callback function for MQTT Subscriptions for publishing messages

        Args:
            client (ActionClient): ROS action client
            userdata (userdata): user data
            message (ros msg): ROS message
        """

        payload = str(message.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)

    def on_goal(self, goal_handle):
        """Called when Action Server receives a Goal

        Args:
            goal_handle (goal handle): goal handle
        """

        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if (goal.protocol == "mqtt" and (
                goal.mode == "pub" or goal.mode == "sub")) or (
                goal.protocol == 'http' and (
                goal.mode == 'email' or goal.mode == 'sheet')):
            goal_handle.set_accepted()

            # Start a new thread to process new goal from the client
            # (For Asynchronous Processing of Goals)
            # 'self.process_goal' - is the function pointer which points to
            # a function that will process incoming Goals
            thread = threading.Thread(name="worker",
                                      target=self.process_goal,
                                      args=(goal_handle,))
            thread.start()

        else:
            goal_handle.set_rejected()
            return

    def process_goal(self, goal_handle):
        """Starts a separate thread to process the Goal

        Args:
            goal_handle (goal handle): goal handle
        """

        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        if goal.protocol == "mqtt":
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(
                    self.mqtt_sub_callback,
                    self._config_mqtt_server_url,
                    self._config_mqtt_server_port,
                    goal.topic,
                    self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        else:
            rospy.logwarn("HTTP")
            result.flag_success = True

            if goal.mode == "sheet":
                params = json.loads(goal.message)

                iot.push_data_to_sheet(
                    self._config_google_apps_spread_sheet_id,
                    **params)
                iot.push_data_to_sheet(
                    self._config_google_apps_spread_sheet_id_eyrc,
                    **params)

        rospy.loginfo("Send goal result to client")
        if result.flag_success:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    # This function will be called when Goal Cancel request is send to the
    # Action Server
    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()


# Main
def main():
    """Main Function
    """

    rospy.init_node('node_action_server_ros_iot_bridge')

    action_server = ActionServerRosIotBridge()

    rospy.spin()


if __name__ == '__main__':
    main()
