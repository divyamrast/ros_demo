#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class SendMotorCommands():

    def __init__(self):
        # Initialize node
        rospy.init_node("move_create")
        self.node_name = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.node_name)
        rospy.on_shutdown(self.shutdown)
        self.rate = 30
        self.odom_received = False
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_x = 0
	self.odom_y = 0
        self.commanded_velocity = 0.5

        # Subscribers
        rospy.Subscriber("odom", Odometry, self.odom_callback)
	time.sleep(0.1)
	self.move_robot()
        rospy.spin()

    def odom_callback(self, data):
        self.odom_x = data.pose.pose.position.x
	self.odom_y = data.pose.pose.position.y	

    def move_robot(self):

        vel_msg = Twist()

        vel_msg.linear.x = self.commanded_velocity
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # Setting the current time for distance calculation
        start_distance = self.odom_x
        current_distance = 0

        # Loop to move create to a specified distance
        while abs(current_distance) < 1:
            print "Current distance is:", current_distance
            # Publish the velocity
            self.velocity_publisher.publish(vel_msg)
            time.sleep(0.05)

            # Calculates distance using odometry
            current_distance = self.odom_x - start_distance

        # After the loop, stops the robot
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Stop the robot
        self.velocity_publisher.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        sendGoal = SendMotorCommands()

    except rospy.ROSInterruptException:
        rospy.loginfo("Command to motor stopped")
