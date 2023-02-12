#!/usr/bin/env python

# Bill Smart, smartw@oregonstate.edu
#
# This example gives the robot callback based driver.


# Import ROS Python basic API and sys
import rospy
import sys

import numpy as np

# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan


def compute_force(distance, charge=1.0, potential_constant=1.0):
    potential = charge * potential_constant / ((distance ** 2) + 0.01)
    return potential


def compute_vector(ranges, min_angle, max_angle, min_dist, max_dist, maintain_distance):
    res = (max_angle - min_angle) / len(ranges)
    vector_sum = np.array([0.0, 0.0])

    # this adds an "incentive" potential for the agent to want to go forward
    vector = np.array([np.cos(np.pi / 2), np.sin(np.pi / 2)])
    potential = compute_force(0.1, potential_constant=1)
    vector *= potential
    vector_sum += vector

    angle_correction = np.pi / 2
    for idx, dist in enumerate(ranges):
        angle = min_angle + (res * idx)
        angle += angle_correction
        dist = min(max_dist, dist)
        dist = max(min_dist, dist)
        if dist <= maintain_distance:
            vector = np.array([np.cos(angle), np.sin(angle)])

            potential = -1 * compute_force(dist, potential_constant=1)
            vector *= potential
            vector_sum += vector

    vector_sum /= (len(ranges) + 1)
    return vector_sum


# A callback to deal with the LaserScan messages.
def callback(scan):
    linear_speed = 0.5
    maintain_dist = 2

    vector = compute_vector(scan.ranges, scan.angle_min, scan.angle_max, scan.range_min, scan.range_max, maintain_dist)

    t = Twist()
    t.linear.x = linear_speed
    t.linear.y = 0
    t.linear.z = 0.0
    t.angular.x = 0
    t.angular.y = 0
    # use the x component to decide how much to turn
    # if the x component is pushing the robot to the right (positive force), then the robot
    # should turn to the right (negative theta)
    # using the x component of the vector also scales the turn by the magnitude of the force
    t.angular.z = -1 * vector[0]

    # Send the command to the robot.
    publisher.publish(t)

    # Print out a log message to the INFO channel to let us know it's working.
    rospy.loginfo(f'Published {t.linear.x=} | {t.angular.z=}')
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

