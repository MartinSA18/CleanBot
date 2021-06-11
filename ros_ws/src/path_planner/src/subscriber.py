#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print (msg.ranges)

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)

# Ensure Python does not exit until node is terminated
rospy.spin()
