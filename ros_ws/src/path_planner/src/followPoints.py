#! /usr/bin/env python

import rospy                               # Import the Python library for ROS
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist             # Import the Int32 message from the std_msgs package
from sensor_msgs.msg import LaserScan


def callback(msg):
    for x in msg.ranges: 
        obstacleDetected = False # Flag ensures all value evaluated before velocity sent.
        if 0 < x < 0.3: # If obstacle within distance, set flag.
            obstacleDetected = True

obstacleDetected = Bool()                          # Create a var of type Int32
rospy.init_node('obstacleDetection')         # Initiate a Node named 'topic_publisher'
pub = rospy.Publisher('/obstacleDetected', Bool, queue_size=10)    
rate = rospy.Rate(100)                       # Set a publish rate of 2 Hz
sub = rospy.Subscriber('/scan', LaserScan, callback)

while not rospy.is_shutdown():             # Create a loop that will go until someone stops the program execution
  pub.publish(obstacleDetected)                       # Publish the message within the 'count' variable 
  rate.sleep()