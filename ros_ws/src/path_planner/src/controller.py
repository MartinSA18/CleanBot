#!/usr/bin/env python

import rospy
import actionlib
from smach import State,StateMachine
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped, PoseArray ,PointStamped, Twist
from std_msgs.msg import Empty
from tf import TransformListener
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
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


from turtlesim.msg import Pose


startOrientationVector = [1, 0]
unit_vector_1 = startOrientationVector / np.linalg.norm(startOrientationVector)
waypoints = []
PI = 3.1415926535897

UPPER_DIST = 0.6
LOWER_DIST = 0.15
VIEWING_ANGLE = 2.269

LINEAR_SPEED = 0.1
ANGULAR_SPEED = 0.5
orientation_tolerance = 0.01
steering_angle = 0

cmd_msg = Twist()

cmd_msg.linear.x = 0.1
cmd_msg.linear.y = 0
cmd_msg.linear.z = 0

cmd_msg.angular.x = 0
cmd_msg.angular.y = 0
cmd_msg.angular.z = 0

goal_waypoint = 0

rospy.init_node('Controller', anonymous=True)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
#current_pose_sub = rospy.Subscriber('/currentPose', Pose2D, queue_size=10) 
#current_pose_sub = rospy.Subscriber('/turtle1/pose', Pose2D, queue_size=10) 
rate = rospy.Rate(10)  

def clean_shutdown(): # Stop robot when Ctrl+C entered
    rospy.loginfo("System is shutting down. Stopping robot...")
    cmd_vel = Twist()  
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0
    cmd_vel_pub.publish(cmd_vel)   

def generateSamplepath(mode):
    startOrientationVector = [1, 0]
    unit_vector_1 = startOrientationVector / np.linalg.norm(startOrientationVector)

    #Mode 1 is just a diagonal line
    if mode == 1:

        for i in range (0,100):
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

        goal_waypoint = waypoints[1]

    #print (waypoints)

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

def get_current_pose():

    #current_pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
    #print("Pose_Data_AMCL:    ")
    #print(current_pose_msg.pose.pose.position.x, "    ", current_pose_msg.pose.pose.position.y)
    #
    #orientation_q = current_pose_msg.pose.pose.orientation
    #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #(roll, pitch, yaw) = euler_from_quaternion(orientation_list)
#
    #pose_to_return = Pose2D()
    #pose_to_return.x = current_pose_msg.pose.pose.position.x
    #pose_to_return.y = current_pose_msg.pose.pose.position.y
    #pose_to_return.theta = yaw



    #current_pose_msg = rospy.wait_for_message('/turtle1/pose', Pose)
    #print("Pose_Data_AMCL:    ")
    #print(current_pose_msg.x, "    ", current_pose_msg.y)
    
    #orientation_q = current_pose_msg.pose.pose.orientation
    #orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #(roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    #pose_to_return = Pose2D()
    #pose_to_return.x = current_pose_msg.x
    #pose_to_return.y = current_pose_msg.y
    #pose_to_return.theta = current_pose_msg.theta


    pose_to_return = rospy.wait_for_message('/currentPose', Pose2D)

    return pose_to_return

def find_nearest_point(robot_pose):
    global waypoints
    #steering_angle = 0

    #last_waypoint_dist = 10000
    #goal_waypoint = waypoints[0]

    for index, waypoint in enumerate(waypoints):

        #if waypoint == waypoints[:-1]:
        #   break
        
        distance = math.sqrt(pow(waypoint.x-robot_pose.x,2)+pow(waypoint.y-robot_pose.y,2))
        
        if distance <= UPPER_DIST and distance >= LOWER_DIST:

            robot_vec_x = math.cos(robot_pose.theta)
            robot_vec_y = math.sin(robot_pose.theta)

            point_vec_x = waypoint.x - robot_pose.x # maybe plus distance to front robot)
            point_vec_y = waypoint.y - robot_pose.y


            dotprod = robot_vec_x * point_vec_x + robot_vec_y * point_vec_y
            norm_robot_vec = math.sqrt(robot_vec_x * robot_vec_x + robot_vec_y * robot_vec_y)
            norm_point_vec = math.sqrt(point_vec_x * point_vec_x + point_vec_y * point_vec_y)

            alpha = math.acos(dotprod/(norm_robot_vec*norm_point_vec))                                 #formula for finding an angle between two vectors

            if alpha <= VIEWING_ANGLE/2:
                goal_waypoint = waypoint
                #rospy.loginfo('Waypoint is found. x: %s, y: %s', goal_waypoint.x, goal_waypoint.y)


                determinant = robot_vec_x * point_vec_y - robot_vec_y * point_vec_x;                      #checking if the aiming vector is on the right hand side or left hand side. (should the cart turn right or left)
                
                if determinant < 0:
                    steering_angle = -alpha

                else:
                    steering_angle = alpha
                    
                if index > 0:
                    waypoints.pop(index-1)

                #return goal_waypoint, steering_angle
                


def execute():  
    #global cmd_vel_pub
    #global orientation_tolerance
    #global ANGULAR_SPEED
    #global LINEAR_SPEED

    
    move_robot = True

    curr_pose = get_current_pose()
    
    
    #goal_waypoint, steering_angle = find_nearest_point(curr_pose)
    find_nearest_point(curr_pose)
    rospy.loginfo('Current position.....   x: %s, y: %s  steer: %s', curr_pose.x, curr_pose.y, steering_angle)
    rospy.loginfo('Going to waypoint....   x: %s,  y: %s', goal_waypoint.x, goal_waypoint.y)

    

    if abs(steering_angle) > orientation_tolerance:
        cmd_msg.angular.z = ANGULAR_SPEED*np.sign(steering_angle)

    if move_robot == True:
        cmd_msg.linear.x = LINEAR_SPEED
        

    rospy.loginfo('Publishing velocities....  x: %s, omega: %s', cmd_msg.linear.x, cmd_msg.angular.z)
    print("   ")

    #except:
    #rospy.loginfo('Could not find waypoint...')# Returned goal waypoint x: %s  y: %s  steer_angle: %s', goal_waypoint.x, goal_waypoint.y, steering_angle)
        


    
    #print(cmd_msg)
    
generateSamplepath(2)
rospy.on_shutdown(clean_shutdown)

while not rospy.is_shutdown():
    execute()
    #print(cmd_msg)
    cmd_vel_pub.publish(cmd_msg)
    #rospy.spin()
    rate.sleep()