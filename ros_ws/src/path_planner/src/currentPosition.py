#!/usr/bin/env python

import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Pose2D, PoseWithCovarianceStamped, PoseArray ,PointStamped, Twist
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

if __name__ == '__main__':
    # initialize node
    rospy.init_node('tf_listener')
    # initalize publisher
    pub = rospy.Publisher('/currentPose', Pose2D, queue_size=10)    
    # Set a publish rate of 60 Hz
    rate = rospy.Rate(60)                       
    # print in console that the node is running
    rospy.loginfo('Started listener node !')
    # create tf listener
    listener = tf.TransformListener()
    # set the node to run 1 time per second (1 hz)
    rate = rospy.Rate(10.0)
    # loop forever until roscore or this node is down
    while not rospy.is_shutdown():
        try:
            # listen to transform
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            # print the transform
            #rospy.loginfo('---------')
            #rospy.loginfo('Translation: ' + str(trans))
            #rospy.loginfo('Rotation: ' + str(rot))
            (roll, pitch, yaw) = euler_from_quaternion(rot)

            currentPose = Pose2D()
            currentPose.x = trans[0]
            currentPose.y = trans[1]
            currentPose.theta = yaw
            #rospy.loginfo('CurrentPose: ' + str(currentPose))
            pub.publish(currentPose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
