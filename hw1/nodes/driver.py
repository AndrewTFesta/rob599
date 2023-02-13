#!/usr/bin/env python3
import argparse
from functools import partial

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
def callback(scan, store_scan, readings_detect, scans_detect):
    """
    Handles receiving a laser scan. More information on this data type can be found at the link below.

    http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html

    This callback parses the scan to find out if the robot detect an object within a particular distance. If it does,
    then it decrements a counter. Once this counter reaches zero, then the robot decides that this detected object is
    real and stops. Note that this callback only looks at a subset of the readings from the middle of the scan.

    Note that there are two effective counters. Once counts how many sensor readings returned that there is an object
    within the detection distance. This counter is computed based on a single scan. If this counter exceeds a particular
    threshold, then this can is marked as detecting an object.

    The second counter counts how many scans returned that there is an object present. Once this counter has detected
    that a certain number of scans are marked as detecting an object, then the robot stops.

    :param scan:            a single laser scan
    :param store_scan:      a list to store the scan detected as having an object
    :param readings_detect: the number of readings from a single scan required to see an object before the scan is
                            considered to detect an object
    :param scans_detect:    the number of scan required to detect an object before the robot stops
    :return:
    """
    linear_speed = 0.5
    min_distance = 1
    scan_width = 10

    # Every time we get a laser scan, calculate the shortest scan distance, and set
    # the speed accordingly.
    mid_point = len(scan.ranges) // 2
    vals = scan.ranges[mid_point - scan_width:mid_point + scan_width]

    # Create a twist and fill in all the fields. For now, we're going to set a constant speed.
    t = Twist()
    t.linear.x = linear_speed
    t.linear.y = 0.0
    t.linear.z = 0.0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = 0

    count_above = sum(each_val <= min_distance for each_val in vals)
    if count_above >= readings_detect:
        store_scan.append(scan)

    if len(store_scan) >= scans_detect:
        # stop
        t.linear.x = 0

    # Send the command to the robot.
    publisher.publish(t)

    # Print out a log message to the INFO channel to let us know it's working.
    rospy.loginfo(f'Published {t.linear.x=}')
    return


if __name__ == '__main__':
    print(f'Starting driver node')
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--scans_detect', default=10, type=int, action='store', required=False)

    args, unknown_args = parser.parse_known_args()
    var_args = vars(args)
    scans_detect = var_args.get('scans_detect', 10)
    readings_detect = var_args.get('readings_detect', 10)

    print(f'--------------------')
    print(f'{args=}')
    print(f'{unknown_args=}')
    print(f'{scans_detect=} | {readings_detect=}')
    print(f'--------------------')

    # Initialize the node, and call it "driver".
    rospy.init_node('driver', argv=sys.argv)

    laser_callback = partial(callback, store_scan=[], readings_detect=readings_detect, scans_detect=scans_detect)

    # Set up a publisher.  The default topic for Twist messages is cmd_vel.
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Set up a subscriber.  The default topic for LaserScan messages is base_scan.
    _ = rospy.Subscriber('base_scan', LaserScan, laser_callback, queue_size=10)
    _ = rospy.Subscriber('scan', LaserScan, laser_callback, queue_size=10)

    # Now that everything is wired up, we just spin.
    rospy.spin()
