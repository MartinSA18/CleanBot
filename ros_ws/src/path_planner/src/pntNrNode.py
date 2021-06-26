#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose, Pose2D, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int32


poseList = []
nextPose = Pose2D()
pntNr = Int32()

for i in range(10): # Creates poseList to iterate through. Will be seperate function in the future.
    generatedPoints = Pose2D()
    generatedPoints.x = 0.2*i
    generatedPoints.y = -0.2*i
    generatedPoints.theta = 0
    poseList.append(generatedPoints)

def pntNrCallback(data):
    if (data.data == 0):
        pntNr.data = 1
    
    else:
        pntNr.data = data.data

def determineGoal(msg, pntNr): # Main function of node
    if pntNr.data == 0:
        pntNr.data = 1

    while pntNr.data < len(poseList)-1:
        print('X =',msg.pose.pose.position.x, 'Y =',msg.pose.pose.position.y, '   pntNr = ', pntNr.data) # Print current position
        currentGoalDist = math.sqrt((msg.pose.pose.position.x - poseList[pntNr.data].x)**2 + (msg.pose.pose.position.y - poseList[pntNr.data].y)**2) # Determine distance from current position to current goal
        pastGoalDist = math.sqrt((msg.pose.pose.position.x - poseList[pntNr.data-1].x)**2 + (msg.pose.pose.position.y - poseList[pntNr.data-1].y)**2) # Determine distance from current position to previous goal
        print(pastGoalDist)
        print(currentGoalDist)
        if pastGoalDist > currentGoalDist:
            pntNr.data+=1
            nextPose.x = poseList[pntNr.data].x
            nextPose.y = poseList[pntNr.data].y
            nextPose.theta = poseList[pntNr.data].theta
            pntNrPub.publish(pntNr.data)
            break
        else:
            nextPose.x = poseList[pntNr.data].x
            nextPose.y = poseList[pntNr.data].y
            nextPose.theta = poseList[pntNr.data].theta
            break
    
rospy.init_node('generatePoints')
pub = rospy.Publisher('/nextPose', Pose2D, queue_size=10)  # Create a 100Hz publisher
pntNrPub = rospy.Publisher('/pntNr', Int32, queue_size=10) # Create pntNr publisher
sub = rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, determineGoal, pntNr) # geometry_msgs/PoseWithCovariance pose
pntNrSub = rospy.Subscriber('/pntNr', Int32, pntNrCallback)

while not rospy.is_shutdown():
   # print(nextPose)
    pub.publish(nextPose)
    rospy.spin()
