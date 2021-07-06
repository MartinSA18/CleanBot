#!/usr/bin/env python

import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Pose2D, PoseWithCovarianceStamped, PoseArray ,PointStamped, Twist
from std_msgs.msg import Empty
from tf import TransformListener
from tf.transformations import quaternion_from_euler
import tf
import math
import rospkg
import csv
import time
from geometry_msgs.msg import PoseStamped
import numpy as np
from visualization_msgs.msg import MarkerArray
import pandas as pd
from shapely.geometry import LineString, Point
from shapely.ops import unary_union
from geopandas import GeoSeries


startOrientationVector = [1, 0]
unit_vector_1 = startOrientationVector / np.linalg.norm(startOrientationVector)
waypoints = []

def clean_shutdown(): # Stop robot when Ctrl+C entered
    rospy.loginfo("System is shutting down. Stopping robot...")
    cmd_vel = Twist()  
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0
    pub.publish(cmd_vel)   


def generateSamplepath(mode):
    startOrientationVector = [1, 0]
    unit_vector_1 = startOrientationVector / np.linalg.norm(startOrientationVector)

    #Mode 1 is just a diagonal line
    if mode == 1:

        for i in range (0,10):
            pose = Pose2D()
            pose.x = 0.1*i
            pose.y = -0.12*i

            vector_2 = [0.1*(i+1) - 0.1*i, 0.12*(i+1) - 0.12*i]
            unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            angle = np.arccos(dot_product)

            pose.theta = angle

            waypoints.append(pose)

    #Mode 2 is a somewhat coverage path (just to show)
    if mode == 2:
        points_x = [0.25,  0.5,  0.5,  0.5,  0.5,  0.5,  0.5,  0.75,  1,    1,    1,    1,    1,  1,  1.25,  1.5,  1.5,  1.5,  1.5,  1.5,  1.5,  1.75,  2,    2,    2,    2,    2,  2,  1.75,  1,  0.1, 0.1]
        points_y_abs = [   0.1,    0.1,  0.4,  0.8,  1.2,  1.6,    2,     2,  2,  1.6,  1.2,  0.8,  0.4,  0.1,     0.1,    0.1,  0.4,  0.8,  1.2,  1.6,    2,     2,  2,  1.6,  1.2,  0.8,  0.4,  0.1,  0.1,  0.1,  0.1, 0.1]
        points_y =  [-i for i in points_y_abs]

        pose1 = Pose2D()
        pose1.x = points_x[0]
        pose1.y = points_y[0]
        pose1.theta = 0
        waypoints.append(pose1)

        for i in range(1, len(points_x)):
            pose = Pose2D()
            pose.x = points_x[i]
            pose.y = points_y[i]

            vector_2 = [points_x[i] - points_x[i-1], points_y[i] - points_y[i-1]]
            unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            angle = np.arccos(dot_product)

            pose.theta = angle

            waypoints.append(pose)

            


    print (waypoints)

def generatePathfromCCP(raw_points, scaling=1):

    startOrientationVector = [1, 0]
    unit_vector_1 = startOrientationVector / np.linalg.norm(startOrientationVector)

    points = raw_points.markers[0].points
    points_x = np.zeros(len(points))
    points_y = np.zeros(len(points))

    for i in range(len(points)):
        points_x[i] = points[i].x/scaling
        points_y[i] = points[i].y/scaling

    for i in range(0,len(points_x)):
        pose = Pose2D()
        pose.x = points_x[i]
        pose.y = points_y[i]

        vector_2 = [points_x[i] - points_x[i-1], points_y[i] - points_y[i-1]]
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        angle = np.arccos(dot_product)

        pose.theta = angle

        waypoints.append(pose)

        if i == len(points)-1:
            goal_pose = raw_points.markers[3].pose
            pose = Pose2D()
            pose.x = goal_pose.position.x
            pose.y = goal_pose.position.y
            pose.theta = angle
            waypoints.append(pose)

def saveCCP(raw_points, write=False, publish=False):
    global startOrientationVector
    global unit_vector_1

    waypoints = []
    points = raw_points.markers[0].points
    path_array = np.zeros((3,len(points)))

    for i in range(len(points)):
        path_array[0,i] = points[i].x
        path_array[1,i] = points[i].y
    
    for i in range(len(points)):

            vector_2 = [path_array[0,i] - path_array[0,i-1], path_array[1,i] - path_array[1,i-1]]
            unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            angle = np.arccos(dot_product)
            path_array[2,i] = angle

            if i == len(points)-1:
                goal_pose = raw_points.markers[3].pose
                path_array[0,i] = goal_pose.position.x
                path_array[1,i] = goal_pose.position.y

    if write:
        path_df=pd.DataFrame(path_array)
        path_df.to_csv("/home/robolab1/Documents/CleanBot/ros_ws/src/path_planner/src/saved_path/path_points.csv", header=False, index=False)

    return path_array


def denseCPP(path_array, raw_points, distance_delta=0.1, write=False):

    global waypoints
    waypoints = []

    path_x=path_array[0]
    path_y=path_array[1]
    
    points = GeoSeries(map(Point, zip(path_x, path_y)))

    line = LineString(points.tolist())
    distances = np.arange(0,line.length, distance_delta)
    points_new = [line.interpolate(distance) for distance in distances] + [line.boundary[1]]

    dense_path = np.zeros((3,len(points_new)))

    for i in range(len(points_new)):
        dense_path[0,i]=points_new[i].x
        dense_path[1,i]=points_new[i].y
    
    ### calculate the thetas

    for i in range(len(points_new)):
        vector_2 = [dense_path[0,i] - dense_path[0,i-1], dense_path[1,i] - dense_path[1,i-1]]
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        angle = np.arccos(dot_product)

        dense_path[2,i] = angle

        pose = Pose2D()
        pose.x = dense_path[0,i]
        pose.y = dense_path[1,i]
        pose.theta = dense_path[2,i]

        waypoints.append(pose)

        
    if write:
        df_dense_path = pd.DataFrame(dense_path)
        df_dense_path.to_csv("/home/robolab1/Documents/CleanBot/ros_ws/src/path_planner/src/saved_path/dense_path.csv", header=False, index=False)


class FollowPath():

    def __init__(self):
        self.frame_id = 'map'
        self.odom_frame_id = 'odom'
        self.base_frame_id = 'base_footprint'
        #self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listner.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = 0.15 #rospy.get_param('waypoint_distance_tolerance', 0.0)
        try:
            rospy.loginfo('Retrieving path from CCP')
            # rospy.init_node('CoverageListener', anonymous=True)
            generatedPoints = rospy.wait_for_message('/coverage_planner/path_markers', MarkerArray)
            # generatePathfromCCP(generatedPoints)
            path_array = saveCCP(generatedPoints, write=True)
            denseCPP(path_array, generatedPoints, distance_delta=0.2,write=True)
        except:
            rospy.loginfo('Could not access CCP')
            rospy.loginfo('Generating sample path...')
            generateSamplepath(2)

    def execute(self):
        self.client.cancel_all_goals()    
        global waypoints
        for waypoint in waypoints[:-1]:
            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue is empty.')
                #rospy.loginfo('Generating sample path....')
                #generateSamplepath()
                break
            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
        # Set waypoint relative to "map" coordinate frame 
            goal.target_pose.pose.position.x = waypoint.x
            goal.target_pose.pose.position.y = waypoint.y
            quaternion = quaternion_from_euler(0, 0, waypoint.theta)
            # Convert yaw to quaternion coordinates
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]

            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %(waypoint.x, waypoint.y))
            
            self.client.send_goal(goal)

            if not self.distance_tolerance > 0.0:
                self.client.wait_for_result()
                rospy.loginfo("Waiting for %f sec..." % self.duration)
                time.sleep(self.duration)
            else:
                #This is the loop which exist when the robot is near a certain GOAL point.
                distance = 2 # used to be 10    
                while(distance > self.distance_tolerance):
                    now = rospy.Time.now()
                    #self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
                    #trans,rot = self.listener.lookupTransform(self.odom_frame_id,self.base_frame_id, now)s
                    current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
                    # rospy.loginfo('Current AMCL Pose: %s, %s,  dist to next goal: %s' %(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, distance))
                    distance = math.sqrt(pow(waypoint.x-current_pose.pose.pose.position.x,2)+pow(waypoint.y-current_pose.pose.pose.position.y,2))
                    rospy.loginfo("\nCurrent goal x:%s y: %s", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)








if __name__ == '__main__':
    rospy.init_node('Waypoints', anonymous=True)
    rospy.on_shutdown(clean_shutdown)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    
    pathfollower = FollowPath()
    print("followpath\n")
    print(waypoints)
    pathfollower.execute()
    rospy.spin()