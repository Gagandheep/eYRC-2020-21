#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pi


class Turtle:
    """
    Turtle class
    Methods: revolve
    """

    def __init__(self):
        """
        Class __init__ function
        """

        # init ros node with name 'node_turtle_revolve'
        rospy.init_node('node_turtle_revolve', anonymous=True)

        # Class attribute of type Pose to hold Turtle's pose data
        self.pose = Pose()
        # Class attribute of type Rate for sleeping in a loop at rate of 10Hz
        self.rate = rospy.Rate(10)
        # Class attribute of Publisher for publishing velocity data
        self.publisher = rospy.Publisher('turtle1/cmd_vel',
                                         Twist,
                                         queue_size=10)
        # Class attribute of Subscriber for getting pose data
        self.subscriber = rospy.Subscriber('turtle1/pose',
                                           Pose,
                                           self.subscriber_callback)

    def subscriber_callback(self, data):
        """
        Callback function for publisher
        """

        # Assign data to self.pose
        self.pose = data
        # Change the range of theta from (-pi, pi] to [0, 2*pi)
        self.pose.theta = round(self.pose.theta
                                + (0. if self.pose.theta >= 0. else 2 * pi),
                                2)

    def revolve(self):
        """
        Function to make the Turtle revolve in a circle CCW
        Arguments: None
        example call: turtle.revolve()
        """

        # Local variable of type Twist
        msg_vel = Twist()
        # Set linear velocity along x axis to 1
        msg_vel.linear.x = 1
        # Set angular velocity about z axis to 1
        msg_vel.angular.z = 1
        # Variable to check whether the Turtle as started to move
        started = False

        # While loop to make the turtle move in a circle
        while not (0 <= self.pose.theta <= 0.05) or not started:
            # Set started to True if theta > 0.05 radians
            if self.pose.theta > 0.05:
                started = True

            # Publish velocity message
            self.publisher.publish(msg_vel)
            # Print/Logging statements
            rospy.loginfo('Moving in a circle')
            rospy.loginfo('{:.2f}'.format(self.pose.theta))
            # Sleep to maintain the rate of 10Hz
            self.rate.sleep()

        # Set linear velocity along x axis to 0
        msg_vel.linear.x = 0
        # Set angular velocity about z axis to 1
        msg_vel.angular.z = 0
        # Publish velocity message
        self.publisher.publish(msg_vel)
        # Print/Logging statement
        rospy.loginfo('goal reached')


def main():
    """
    main function
    """

    # Create instance of class Turtle
    turtle = Turtle()
    # Invoke revolve method
    turtle.revolve()

    # Keep python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print e
