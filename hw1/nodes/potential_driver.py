#!/usr/bin/env python
import argparse
from functools import partial

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


def compute_force(distance, potential_constant=1.0):
    potential = potential_constant / ((distance ** 2) + 0.01)
    return potential


def compute_vector(ranges, initial_angle, angle_resolution, min_dist, max_dist, maintain_distance):
    """
    Computes the vector field surrounding the robot based on the range of distance readings surrounding the robot.

    :param ranges:
    :param initial_angle:
    :param angle_resolution:
    :param min_dist:
    :param max_dist:
    :param maintain_distance:
    :param min_angle:
    :param max_angle:
    :return:
    """
    potential_constant = 20
    # res = (max_angle - min_angle) / len(ranges)
    vector_sum = np.array([0.0, 0.0])

    # this adds an "incentive" potential for the agent to want to go forward
    vector = np.array([np.cos(np.pi / 2), np.sin(np.pi / 2)])
    potential = compute_force(0.1, potential_constant=potential_constant)
    vector *= potential
    vector_sum += vector

    angle_correction = np.pi / 2
    for idx, dist in enumerate(ranges):
        angle = initial_angle + (angle_resolution * idx)
        dist = min(max_dist, dist)
        dist = max(min_dist, dist)

        if dist <= maintain_distance:
            angle += angle_correction
            vector = np.array([np.cos(angle), np.sin(angle)])

            potential = -1 * compute_force(dist, potential_constant=potential_constant)
            vector *= 1 / (dist + 0.01)
            vector *= potential
            vector_sum += vector

    vector_sum /= (len(ranges) + 1)
    return vector_sum


# A callback to deal with the LaserScan messages.
def callback(scan, fudge_factor, store_scan, store_len):
    """
    Handles receiving a laser scan. More information on this data type can be found at the link below.

    http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html

    :param scan:            a single laser scan
    :param fudge_factor:    allows for biasing the turning of the robot in one direction or another. a positive value
                            will push the robot to turn left and a negative value will push the robot to turn right.
    :param store_scan:      a list where this scan can be saved (for persistence)
    :param store_len:       the maximum number of scans to store in store_scan
    :return:
    """
    linear_speed = 0.5
    maintain_dist = 2

    vector = compute_vector(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max, maintain_dist)
    print(vector)

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
    turn_amount = -1 * vector[0]
    turn_amount += fudge_factor
    t.angular.z = turn_amount

    # Send the command to the robot.
    publisher.publish(t)

    # Print out a log message to the INFO channel to let us know it's working.
    # rospy.loginfo(f'Published {t=}')
    store_scan.append(scan)
    store_scan = store_scan[-store_len:]
    return store_scan


if __name__ == '__main__':
    print(f'Starting potential driver node')
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--fudge_factor', default=0, type=int, action='store', required=False)
    parser.add_argument('--store_len', default=10, type=int, action='store', required=False)

    args, unknown_args = parser.parse_known_args()
    var_args = vars(args)
    fudge_factor = var_args.get('fudge_factor', 0)
    store_len = var_args.get('store_len', 10)

    print(f'--------------------')
    print(f'{args=}')
    print(f'{unknown_args=}')
    print(f'{fudge_factor=} | {store_len=}')
    print(f'--------------------')

    # Initialize the node, and call it "driver".
    rospy.init_node('driver', argv=sys.argv)

    laser_callback = partial(callback, fudge_factor=fudge_factor, store_scan=[], store_len=store_len)

    # Set up a publisher.  The default topic for Twist messages is cmd_vel.
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Set up a subscriber.  The default topic for LaserScan messages is base_scan.
    _ = rospy.Subscriber('base_scan', LaserScan, laser_callback, queue_size=10)
    _ = rospy.Subscriber('scan', LaserScan, laser_callback, queue_size=10)

    print('Time keeps on spinning...')
    # Now that everything is wired up, we just spin.
    rospy.spin()
