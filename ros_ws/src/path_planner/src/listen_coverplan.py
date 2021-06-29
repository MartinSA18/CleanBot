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


    










if __name__ == '__main__':
    rospy.init_node('CoverageListener', anonymous=True)
    waypoints = rospy.wait_for_message('/coverage_planner/path_markers', MarkerArray)
    # print(type(waypoints))
    
    scaling = 100
    path_points = []
    startOrientationVector = [1, 0]
    unit_vector_1 = startOrientationVector / np.linalg.norm(startOrientationVector)


    points = waypoints.markers[0].points
    print(points)
    print(len(points))
    print(points[0].x)
    points_x = np.zeros(len(points))
    points_y = np.zeros(len(points))

    for i in range(len(points)):
        print(i)
        points_x[i] = points[i].x/scaling
        points_y[i] = -points[i].y/scaling

    for i in range(len(points_x)):
            pose = Pose2D()
            pose.x = points_x[i]
            pose.y = points_y[i]

            vector_2 = [points_x[i] - points_x[i-1], points_y[i] - points_y[i-1]]
            unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            angle = np.arccos(dot_product)

            pose.theta = angle

            path_points.append(pose)

    print(path_points)
    print(len(path_points))


    rospy.spin()
    