#! /usr/bin/env python

import rospy                               # Import the Python library for ROS
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

def callback(msg):
    obstacleDetected.data = False # Flag ensures all value evaluated before boolean sent.
    for x in msg.ranges: 
        if 0 < x < 0.2: # If obstacle within distance, set flag.
            obstacleDetected.data = True
    print(obstacleDetected)
            
obstacleDetected = Bool()                          # Create a var of type Bool
rospy.init_node('obstacleDetection')         # Initiate a Node named 'obstacleDetection'
pub = rospy.Publisher('/obstacleDetected', Bool, queue_size=10)  # Create a 100Hz publisher 
rate = rospy.Rate(100)                       
sub = rospy.Subscriber('/scan', LaserScan, callback) # Create subscriber    

while not rospy.is_shutdown():             # Create a loop that will go until someone stops the program execution
  print(obstacleDetected)
  pub.publish(obstacleDetected)                       # Publish the message within the 'count' variable 
  rate.sleep()