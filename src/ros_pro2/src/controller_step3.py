#!/usr/bin/python3

import rospy 
import tf
from math import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import numpy as np
import math

class Controller:

    def __init__(self) -> None:
        rospy.init_node("controller", anonymous=False)

        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, callback=self.odom_callback)
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

        self.distance_star = -0.02

        self.goal = Point()
        self.g_index = 0

        self.d_integral = 0
        self.a_integral = 0

        self.time = rospy.get_time()

        self.distance = 0
        self.angle = 0

        self.x_inputs = np.array([])
        self.y_inputs = np.array([])

        

        self.set_shape()
        self.start()

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

        
    def start(self):
        goal = Point()
        msg = rospy.wait_for_message("/odom", Odometry)
        position = msg.pose.pose.position
        for i in range(self.x_inputs.size):
            error = [sqrt(pow(self.x_inputs[i]-position.x, 2) + pow(self.y_inputs[i] - position.y, 2))]
        
        nearest_point = min(enumerate(error), key=lambda x: x[1])[0]

        goal.x = self.x_inputs[nearest_point]
        goal.y = self.y_inputs[nearest_point]

        self.g_index = nearest_point
        self.goal = goal

    def odom_callback(self, msg: Odometry):
        twist = Twist()
        distance, angle = self.get_euler(msg)
        curr_time = rospy.get_time()

        d_time = curr_time - self.time
        self.time = curr_time

        self.d_integral = self.d_integral + (d_time*distance)
        self.a_integral = self.a_integral + (d_time*angle)

        dis = distance - self.distance
        ang = angle - self.angle

        if distance <= self.stop_distance:
            self.next_goal()
        else:
            linear_speed = (self.kp_distance*distance) + (self.ki_distance*self.d_integral) + (self.kd_distance*dis)
            if linear_speed < 0:
                twist.linear.x = max(linear_speed, -self.linear_speed)
            else:
                twist.linear.x = min(linear_speed, self.linear_speed)

            angular_speed = (self.kp_angle*angle) + (self.ki_angle*self.a_integral) + (self.kd_angle*ang)
            if angular_speed < 0:
                twist.angular.z = max(angular_speed, -self.angular_speed)
            else:
                twist.angular.z = min(angular_speed, self.angular_speed)

            self.distance = dis
            self.angular = ang

            self.cmd_publisher.publish(twist)

    def get_euler(self, msg: Odometry):
        curr_position = msg.pose.pose.position
        curr_orientation = msg.pose.pose.orientation

        pose = msg.pose.pose
        diff_x = self.goal.x - curr_position.x
        diff_y = self.goal.y - curr_position.y

        distance = sqrt(pow(diff_x,2)+pow(diff_y,2)) + self.distance_star
        if distance < 0:
            distance = 0

        # Rotation
        yaws = atan2(self.goal.y - curr_position.y, self.goal.x - curr_position.x)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            curr_orientation.x, curr_orientation.y, curr_orientation.z, curr_orientation.w
        ))

        if yaws < 0:
            yaws += radians(360)
        if yaw < 0:
            yaw += radians(360)

        orientation = 0
        if yaws > yaw and yaws - yaw > radians(360) - (yaws - yaw):
            orientation = -(radians(360) - (yaws - yaw))
        elif yaws > yaw and yaws - yaw < radians(360) - (yaws - yaw):
            orientation = yaws - yaw
        elif yaw > yaws and yaw - yaws > radians(360) - (yaw - yaws):
            orientation = radians(360) - (yaw - yaws)
        elif yaw > yaws and yaw - yaws < radians(360) - (yaw - yaws):
            orientation = -(yaw - yaws)

        return distance, orientation

    def next_goal(self):
        goal = Point()
        self.g_index = (self.g_index + 1) % self.x_inputs.size
        goal.x = self.x_inputs[self.g_index]
        goal.y = self.y_inputs[self.g_index]
        self.goal = goal

if __name__ == "__main__":
    controller = Controller()
    rospy.spin()