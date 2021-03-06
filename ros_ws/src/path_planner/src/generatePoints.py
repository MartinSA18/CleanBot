#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose, Pose2D, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

poseList = []
nextPose = Pose2D()
pntNr = 1

for i in range(10): # Creates poseList to iterate through. Will be seperate function in the future.
    generatedPoints = Pose2D()
    generatedPoints.x = 0.2*i
    generatedPoints.y = -0.2*i
    generatedPoints.theta = 0
    poseList.append(generatedPoints)

def determineGoal(msg, pntNr): # Main function of node
    while pntNr < len(poseList)-1:
        print('X =',msg.pose.pose.position.x, 'Y =',msg.pose.pose.position.y, '   pntNr = ', pntNr) # Print current position
        currentGoalDist = math.sqrt((msg.pose.pose.position.x - poseList[pntNr].x)**2 + (msg.pose.pose.position.y - poseList[pntNr].y)**2) # Determine distance from current position to current goal
        pastGoalDist = math.sqrt((msg.pose.pose.position.x - poseList[pntNr-1].x)**2 + (msg.pose.pose.position.y - poseList[pntNr-1].y)**2) # Determine distance from current position to previous goal
        print(pastGoalDist)
        print(currentGoalDist)
        if pastGoalDist > currentGoalDist:
            pntNr+=1
            nextPose.x = poseList[pntNr].x
            nextPose.y = poseList[pntNr].y
            nextPose.theta = poseList[pntNr].theta
            break
        else:
            nextPose.x = poseList[pntNr].x
            nextPose.y = poseList[pntNr].y
            nextPose.theta = poseList[pntNr].theta
            break
    
rospy.init_node('generatePoints')
pub = rospy.Publisher('/nextPose', Pose2D, queue_size=10)  # Create a 100Hz publisher
sub = rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, determineGoal, pntNr) # geometry_msgs/PoseWithCovariance pose

while not rospy.is_shutdown():
   # print(nextPose)
    pub.publish(nextPose)
    rospy.spin()
