#!/usr/bin/env python
import rospy
import math

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Pose2D, Point, Quaternion
from tf.transformations import quaternion_from_euler

def movebase_client(newPose):

   # Create an action client called "move_base" with action definition file "MoveBaseAction". Wait for initialization.
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Set waypoint relative to "map" coordinate frame 
    goal.target_pose.pose.position.x = newPose.x
    goal.target_pose.pose.position.y = newPose.y
    quaternion = quaternion_from_euler(0, 0, newPose.theta)
    # Convert yaw to quaternion coordinates
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

   # Sends the goal to the action server and waits for result
    client.send_goal(goal)
    wait = client.wait_for_result()

   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
rospy.init_node('followPoints')
sub = rospy.Subscriber('/nextPose', Pose2D, movebase_client)
rospy.spin()