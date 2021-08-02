#!/usr/bin/env python

# ROS Node - Simple Action Client - Turtle

import rospy
import actionlib

# Message Class imports
from pkg_task1.msg import msgTurtleAction  # used by ROS Actions internally
from pkg_task1.msg import msgTurtleGoal  # used for Goal messages
from pkg_ros_iot_bridge.msg import msgRosIotAction  # used by ROS Actions
from pkg_ros_iot_bridge.msg import msgRosIotGoal  # Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult  # Result Messages
from pkg_ros_iot_bridge.msg import msgMqttSub  # MQTT Subscription Messages


class IoTActionClientTurtle:

    # Constructor
    def __init__(self):
        self._ac_turtle = actionlib.SimpleActionClient('/action_turtle',
                                                       msgTurtleAction)
        self._ac_ros_iot = actionlib.SimpleActionClient('/action_ros_iot',
                                                        msgRosIotAction)
        self._ac_turtle.wait_for_server()
        self._ac_ros_iot.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goals!")

        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']

    # Function to send Goals to Action Servers
    def send_goal(self, arg_dis, arg_angle):
        # Create Goal message for Simple Action Server
        goal_turtle = msgTurtleGoal(distance=arg_dis, angle=arg_angle)

        '''
        * done_cb is set to the function pointer of the function which 
        should be called once the Goal is processed by the Simple Action 
        Server.

        * feedback_cb is set to the function pointer of the function which 
        should be called while the goal is being processed by the Simple 
        Action Server.
        '''
        self._ac_turtle.send_goal(goal_turtle,
                                  done_cb=self.done_callback,
                                  feedback_cb=self.feedback_callback)

        rospy.loginfo("Goal has been sent.")
        self._ac_turtle.wait_for_result()
        result = self._ac_turtle.get_result()
        result = (result.final_x, result.final_y, result.final_theta)
        goal_ros_iot = msgRosIotGoal('mqtt',
                                     'pub',
                                     self._config_mqtt_pub_topic,
                                     str(result))
        self._ac_ros_iot.send_goal(goal_ros_iot)

    # Function print result on Goal completion
    def done_callback(self, status, result):
        rospy.loginfo("Status is : " + str(status))
        rospy.loginfo("Result is : " + str(result))

    # Function to print feedback while Goal is being processed
    def feedback_callback(self, feedback):
        rospy.loginfo(feedback)

    # Function to trace a hexagon with specified side length
    def trace_hexagon(self, side):
        self.send_goal(side, 0)
        self._ac_turtle.wait_for_result()
        for _ in range(5):
            self.send_goal(side, 60)
            self._ac_turtle.wait_for_result()


# Subscriber Callback
def subs_callback(data, client):
    if data.message == 'start':
        client.trace_hexagon(2)


# Main Function
def main():
    # 1. Initialize ROS Node
    rospy.init_node('node_iot_action_client_turtle')

    # 2. Create a object for Simple Action Client.
    client = IoTActionClientTurtle()

    # Subscribe to the topic /ros_iot_bridge/mqtt/sub
    subs = rospy.Subscriber('/ros_iot_bridge/mqtt/sub',
                            msgMqttSub,
                            subs_callback,
                            client)

    # 4. Loop forever
    rospy.spin()


if __name__ == '__main__':
    main()
