#!/usr/bin/python3

from matplotlib import pyplot as plt
import numpy as np
import rospy
import tf
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

from math import atan2, pi, radians, sqrt

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        r = rospy.Rate(10)
        sub = rospy.Subscriber("/odometry/filtered", Odometry, self.get_heading)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.stop_distance = rospy.get_param("/controller/stop_distance") # m
        self.epsilon = rospy.get_param("/controller/epsilon")
        self.distanceThreshold = 0.3
        
        # Define PID parameters
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        self.kp_distance = 0.4
        self.ki_distance = 0.04
        self.kd_distance = 2

        self.kp_angle = 1
        self.ki_angle = 0.03
        self.kd_angle = 0.05

        self.previous_angle = 0
        self.total_angle = 0
        self.previous_distance = 0

        self.errors =[]
        self.shape = []
        
    def get_heading(self, euler = True):
        
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        if euler:
            return yaw
        return  msg.pose.pose.position

    def rotate(self):
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)

        remaining = self.goal_angle
        prev_angle = self.get_heading()
        
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)
        
        while remaining >= self.epsilon:
            current_angle = self.get_heading()
            delta = abs(prev_angle - current_angle)
            remaining -= delta
            prev_angle = current_angle
        
        self.cmd_publisher.publish(Twist())

    def move_straight(self):
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.cmd_publisher.publish(twist)

    def plan_move(self,p2, threshold):
        while True:
            current_pose = self.get_heading(False)
            self.errors.append(get_error(self.rectangle, current_pose))
            if check_distance(current_pose, p2, threshold=threshold):
                rospy.loginfo("Moving Straight")
                self.move_straight()
            else:
                rospy.loginfo("Roatating")
                self.rotate()
                return            

    def run(self):

        origin = Point()
        origin.x = 2
        origin.y = 0
        total_distance = 0
        last_rotation = 0
    
        rectangle = []
        X1 = np.linspace(-2, 2 , 10)

        for x in X1:
            rectangle.append([x,3])


        Y2 = np.linspace(3, -3 , 10)
        X2 = np.array([3]*100)

        for y in Y2:
            rectangle.append([2,y])

        X3 = np.linspace(2, -2 , 10)
        Y3 = np.array([-2]*100)

        for x in X3:
            rectangle.append([x,-3])

        Y4 = np.linspace(-3, 3 , 10)
        X4 = np.array([-3]*10)

        for y in Y4:
            rectangle.append([-2,y])

        self.rectangle = rectangle

        twist = Twist()
        while not rospy.is_shutdown():
            
            for goal in self.rectangle:
                current_pose = self.get_heading(False)
                distance = sqrt(pow((goal[0] - current_pose.x), 2) + pow((goal[1] - current_pose.y), 2)) - 0.05
                rospy.loginfo(distance)

                
                self.errors.append(get_error(self.rectangle, current_pose))
                
                while distance > 0.05:
                    rotation = self.get_heading(True)
                    
                    path_angle = atan2(goal[1]-current_pose.y , goal[0] - current_pose.x) 
                    
                    if path_angle < -pi/4 or path_angle > pi/4:
                        if goal[1] < 0 and current_pose.y < goal[1]:
                            path_angle = -2*pi + path_angle
                    if last_rotation > pi-0.1 and rotation <= 0:
                        rotation = 2*pi + rotation
                    elif last_rotation < -pi+0.1 and rotation > 0:
                        rotation = -2*pi + rotation
                    elif rotation > 0 and goal[0] < 0 and goal[1] < 0:
                        rotation = -2*pi + rotation

                    #diff_angle = path_angle - self.previous_angle
                    #diff_distance = distance - self.previous_distance
                    
                    distance = sqrt(pow((goal[0] - current_pose.x), 2) + pow((goal[1] - current_pose.y), 2)) - 0.05

                    current_pose = self.get_heading(False)
                    self.errors.append(get_error(self.rectangle, current_pose))
                    
                    control_signal_distance = self.kp_distance*distance + self.ki_distance*total_distance # +self.kd_distance*diff_distance

                    control_signal_angle = self.kp_angle*(path_angle-rotation)  # + self.ki_angle*self.total_angle + self.kd_distance*diff_angle

                    twist.angular.z = (control_signal_angle) 
                    twist.linear.x = min(control_signal_distance, 0.1)

                    if twist.angular.z > 0:
                        twist.angular.z = min(twist.angular.z, 1.5)
                    else:
                        twist.angular.z = max(twist.angular.z, -1.5)
                    
                    self.cmd_publisher.publish(twist)
                    
                    last_rotation = rotation
                    total_distance += distance
                    self.previous_distance = distance


def get_distance(p1, p2):
    return sqrt((int(p1.x) - int(p2.x)) ** 2 + (int(p1.y) - int(p2.y)) ** 2)

def check_distance(p1,p2, threshold):
    if get_distance(p1,p2) <= threshold:
        return True

def get_error(rectangle,p2):
    distances = []
    for p in rectangle:
        distances.append(sqrt((int(p[0]) - int(p2.x)) ** 2 + (int(p[1]) - int(p2.y)) ** 2))

    return np.argmin(np.asarray(distances))

def show_plot(error):
    plt.figure()
    plt.plot(error,range(len(error)))
    plt.xlabel('points')
    plt.ylabel('error')
    plt.legend()
    plt.title('Following Path Error')
    plt.show()

if __name__ == "__main__":
    try:
        controller = Controller()
        controller.run()
    except rospy.ROSInterruptException:
        show_plot(controller.errors)
