#!/usr/bin/env python

from os import wait
from pandas.io.pytables import IndexCol
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

if __name__ == '__main__':
    rospy.init_node('CoverageGenerator', anonymous=True)
    raw_points = rospy.wait_for_message('/coverage_planner/path_markers', MarkerArray)
    path_array = saveCCP(raw_points, write=True)
    denseCPP(path_array, raw_points, write=True)
    print(waypoints)
    rospy.spin()
    