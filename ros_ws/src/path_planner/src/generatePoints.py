#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose, Pose2D, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

poseList = []
pntNr = 1
for i in range(10): # Creates poseList to interate through. Will be seperate function in the future.
    nextPose = Pose2D()
    if i >= 0:
        poseList[i].position.x = 0.1*i
        poseList[i].position.y = 0.1*i
        poseList[i].position.theta = 0
    poseList.append(pose)

def determineGoal (msg): # Main function of node
    while pntNr <= len(poseList):
        print('X =',msg.pose.pose.position.x, 'Y =',msg.pose.pose.position.y, 'Yaw =',math.degrees(yaw)) # Print current position
        currentGoalDist = sqrt((msg.pose.pose.position.x - poseList[pntNr].x)**2 + (msg.pose.pose.position.y - poseList[pntNr].y)**2) # Determine distance from current position to current goal
        pastGoalDist = sqrt((msg.pose.pose.position.x - poseList[pntNr-1].x)**2 + (msg.pose.pose.position.y - poseList[pntNr-1].y)**2) # Determine distance from current position to previous goal
        if pastGoalDist > currentGoalDist:
            pntNr+=1
    
rospy.init_node('generatePoints')
pub = rospy.Publisher('/nextPose', Pose2D, queue_size=10)  # Create a 100Hz publisher
r = rospy.Rate(100)
sub = rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, determineGoal) # geometry_msgs/PoseWithCovariance pose

while not rospy.is_shutdown():
    print(nextPose)
    pub.publish(nextPose)
    rate.sleep()