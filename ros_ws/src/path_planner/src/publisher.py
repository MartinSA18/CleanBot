#! /usr/bin/env python

import rospy                               # Import the Python library for ROS
from geometry_msgs.msg import Twist             # Import the Int32 message from the std_msgs package
from std_msgs.msg import Bool

def clean_shutdown(): # Stop robot when Ctrl+C entered
    rospy.loginfo("System is shutting down. Stopping robot...")
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0
    pub.publish(cmd_vel)   

def callback(msg):
    if  msg == True: # If obstacle stop robot, else continue.
        cmd_vel.linear.x = 0.0                             
        cmd_vel.angular.z = 0.0
    else:
        cmd_vel.linear.x = 0.2
        cmd_vel.angular.z = 0

rospy.on_shutdown(clean_shutdown)
cmd_vel = Twist()                          # Create a var of type Int32
rospy.init_node('vel_publisher')         # Initiate a Node named 'vel_publisher'
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
rate = rospy.Rate(100)                       # Set a publish rate of 100 Hz
sub = rospy.Subscriber('/obstacleDetected', Bool, callback)

while not rospy.is_shutdown():             # Create a loop that will go until someone stops the program execution
  pub.publish(cmd_vel)                       # Publish the message within the 'count' variable 
  rate.sleep()