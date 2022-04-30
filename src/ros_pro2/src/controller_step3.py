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
           
        self.twist = Twist()

        #Define PID parameters
        self.kp_distance = 0.5
        self.ki_distance = 0.0001
        self.kd_distance = 0.001

        self.kp_angle = 1
        self.ki_angle = 0
        self.kd_angle = 0

        self.goal = Point()
        self.g_index = 0

        self.d_integral = 0
        self.a_integral = 0

        self.time = rospy.get_time()

        self.distance = 0
        self.angle = 0

        self.x_inputs = np.array([])
        self.y_inputs = np.array([])

        print("Please enter shape number for the following path: ")
        print("1. logarithmic spiral")
        print("2. two half circle")
        print("3. archimedean spiral")
        print("4. octagonal")
        shape_number = input()

        self.set_shape(shape_number)

    def set_shape(number):
        if number == 1:
            a = 0.17
            k = math.tan(a)
            X , Y = [] , []
            for i in range(150):
                t = i / 20 * math.pi
                dx = a * math.exp(k * t) * math.cos(t)
                dy = a * math.exp(k * t) * math.sin(t)
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

    def start(self):
        goal = Point()
        msg = rospy.wait_for_message("/odom", Odometry)
        position = msg.pose.position
        for i in range(self.x_inputs.size):
            error = [sqrt(pow(self.x_inputs[i]-position.x, 2) + pow(self.y_inputs[i] - position.y, 2))]
        
        nearest_point = min(enumerate(error), key=lambda x: x[1])[0]

        goal.x = self.x_inputs[nearest_point]
        goal.y = self.y_inputs[nearest_point]

        self.g_index = nearest_point
        self.goal = goal
if __name__ == "__main__":
    controller = Controller()
    rospy.spin()