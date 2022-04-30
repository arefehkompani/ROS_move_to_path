#!/usr/bin/python3

import rospy 
import tf
from math import *
from nav_msgs import Odometry
from geometry_msgs.msg import Twist, Point
import numpy as np

class Controller:

    def __init__(self) -> None:
        rospy.init_node("controller", anonymous=False)

        self.odom_subscriber = rospy.Subscriber("/odom", Odometry)
        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.linear_speed = rospy.get_param("/controller/linear_speed")     
        self.angular_speed = rospy.get_param("/controller/angular_speed")  
        self.stop_distance = rospy.get_param("/controller/stop_distance")
        self.epsilon = rospy.get_param("/controller/epsilon")   
