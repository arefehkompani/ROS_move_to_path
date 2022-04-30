#!/usr/bin/python3

from matplotlib import pyplot as plt
import numpy as np
import rospy
import tf

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

from math import radians, sqrt

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        sub = rospy.Subscriber("/odometry/filtered", Odometry, self.get_heading)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.stop_distance = rospy.get_param("/controller/stop_distance") # m
        self.epsilon = rospy.get_param("/controller/epsilon")
        self.distanceThreshold = 0.3
        
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        self.errors =[]
        self.rectangle = []
        
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
        rospy.sleep(1)

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
                rospy.loginfo("---Roatating---")
                self.rotate()
                return            

    def run(self):

        origin = Point()
        origin.x = 2
        origin.y = 0
    
        rectangle = []
        X1 = np.linspace(-2, 2 , 100)

        for x in X1:
            rectangle.append([x,3])


        Y2 = np.linspace(3, -3 , 100)
        X2 = np.array([3]*100)

        for y in Y2:
            rectangle.append([2,y])

        X3 = np.linspace(2, -2 , 100)
        Y3 = np.array([-2]*100)

        for x in X3:
            rectangle.append([x,-3])

        Y4 = np.linspace(-3, 3 , 100)
        X4 = np.array([-3]*100)

        for y in Y4:
            rectangle.append([-2,y])

        self.rectangle = rectangle

        while True:
            current_pose = self.get_heading(False)
            if sqrt((int(current_pose.x) - origin.x) ** 2 + (int(current_pose.y) - origin.y) ** 2) >= self.distanceThreshold:
                rospy.loginfo("Moving Forward")
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_publisher.publish(twist)
            else:
                rospy.loginfo("---Roatating---")
                self.rotate()
                break

        while not rospy.is_shutdown():

            origin.x = 2
            origin.y = 3
            self.plan_move(origin, 6)

            origin.x = 2
            origin.y = -3
            self.plan_move(origin, 4)

            origin.x = -2
            origin.y = -3
            self.plan_move(origin, 6)

            origin.x = -2
            origin.y = 3
            self.plan_move(origin, 4)
            
            self.state = self.GO


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
