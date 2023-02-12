#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# This example gives the robot callback based driver.


# Import ROS Python basic API and sys
import rospy
import sys


# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan


# A callback to deal with the LaserScan messages.
def callback(scan):
        linear_speed = 0.5
        turn_speed = 2
        min_distance = 1
        scan_width = 10

        # Every time we get a laser scan, calculate the shortest scan distance, and set
        # the speed accordingly.
        mid_point = len(scan.ranges) // 2
        vals = scan.ranges[mid_point - scan_width:mid_point + scan_width]


        # Create a twist and fill in all the fields.  For now, we're going to set a
        # constant speed.
        t = Twist()
        t.linear.x = linear_speed
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = 0

        if any(each_val <= min_distance for each_val in vals):
            # stop and turn left
            t.linear.x = 0
            # t.angular.z = turn_speed

        # Send the command to the robot.
        publisher.publish(t)

        # Print out a log message to the INFO channel to let us know it's working.
        rospy.loginfo(f'Published {t.linear.x=}')
        return


if __name__ == '__main__':
        # Initialize the node, and call it "driver".
        rospy.init_node('driver', argv=sys.argv)

        # Set up a publisher.  The default topic for Twist messages is cmd_vel.
        publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Set up a subscriber.  The default topic for LaserScan messages is base_scan.
        subscriber = rospy.Subscriber('base_scan', LaserScan, callback, queue_size=10)

        # Now that everything is wired up, we just spin.
        rospy.spin()
