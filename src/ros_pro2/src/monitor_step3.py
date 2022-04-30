#!/usr/bin/python3

import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
from math import *
import math

class Monitor:
    def __init__(self) -> None:
        rospy.init_node("monitor", anonymous=False)

        self.error = []
        self.time_error = []

        self.x_inputs = np.array([])
        self.y_inputs = np.array([])


        self.set_shape()

        self.path = Path()

        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        self.path_publisher = rospy.Publisher("/path" , Path , queue_size=10)

    def set_shape(self):
        print("Please enter shape number for the following path: ")
        print("1. logarithmic spiral")
        print("2. two half circle")
        print("3. archimedean spiral")
        print("4. octagonal")
        number  = input()
        
        if number == 1:
            a = 0.17
            k = tan(a)
            X , Y = [] , []
            for i in range(150):
                t = i / 20 * pi
                dx = a * exp(k * t) * cos(t)
                dy = a * exp(k * t) * sin(t)
                X.append(dx)
                Y.append(dy) 
            self.x_inputs = np.array(X)
            self.y_inputs = np.array(Y)
        elif number == 2:
            
            X1 = np.linspace(-6., -2 , 50)
            Y1 = np.zeros((50,))

            x_dim, y_dim = 2,2
            t = np.linspace(np.pi, 0, 100)
            X2 = x_dim * np.cos(t) 
            Y2 = y_dim * np.sin(t)

            X3 = np.linspace(2, 6 , 50)
            Y3 = np.zeros((50,))

            x_dim, y_dim = 6,6
            t = np.linspace(np.pi*2, np.pi, 200)
            X4 = x_dim * np.cos(t) 
            Y4 = y_dim * np.sin(t)

            X = np.concatenate([X1, X2, X3, X4])
            Y = np.concatenate([Y1, Y2, Y3, Y4])

            self.x_inputs = X
            self.y_inputs = Y
        
        elif number == 3:
            growth_factor = 0.1
            X , Y = [] , []

            for i in range(400):
                t = i / 20 * math.pi
                dx = (1 + growth_factor * t) * math.cos(t)
                dy = (1 + growth_factor * t) * math.sin(t)
                X.append(dx)
                Y.append(dy)
            self.x_inputs = np.array(X)
            self.y_inputs = np.array(Y)

        else: 
            X1 = np.linspace(-1, 1 , 100)
            Y1 = np.array([3]*100)

            X2 = np.linspace(1, 1 + 2**(1/2) , 100)
            Y2 = - (2**(1/2)) * (X2 - 1) + 3

            Y3 = np.linspace(1, -1 , 100)
            X3 = np.array([1 + 2**(1/2)]*100)

            X4 = np.linspace(1 + 2**(1/2), 1, 100)
            Y4 = (2**(1/2)) * (X4 - 1 - 2**(1/2)) -1 

            X5 = np.linspace(1, -1 , 100)
            Y5 = np.array([-3]*100)

            X6 = np.linspace(-1, -1 - 2**(1/2) , 100)
            Y6 = - (2**(1/2)) * (X6 + 1) - 3 


            Y7 = np.linspace(-1, 1 , 100)
            X7 = np.array([- 1 - 2**(1/2)]*100)


            X8 = np.linspace(-1 - 2**(1/2), -1, 100)
            Y8 = (2**(1/2)) * (X8 + 1 + 2**(1/2)) + 1

            X = np.concatenate([X1,X2,X3,X4,X5,X6,X7,X8])
            Y = np.concatenate([Y1,Y2,Y3,Y4,Y5,Y6,Y7,Y8])

            self.x_inputs = X
            self.y_inputs = Y

    def odom_callback(self, msg: Odometry):
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header

        pose.pose = msg.pose.pose
        self.time_error.append(rospy.get_time())
        self.error.append(self.find_error(msg.pose.pose.position))
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

    def find_error(self, point: Point):
        for i in range(self.x_inputs.size):
            minimum = min([sqrt(pow(self.x_inputs[i]-point.x, 2) + pow(self.y_inputs[i] - point.y, 2))])
            return minimum
    
    def plotit(self):
        plt.xlabel('Time')
        plt.ylabel('error')
        plt.plot(self.time_error, self.error)
        plt.show()

if __name__=="__main__":
    monitor = Monitor()
    rospy.on_shutdown(monitor.plotit)
    rospy.spin()