#!/usr/bin/env python

# ROS general imports
import roslib
roslib.load_manifest('lab1_turtlebot')
import rospy

# Other imoprts
import math #math library
import numpy as np #numpy library
from probabilistic_lib.functions import angle_wrap #Normalize angles between -pi and pi

#TODO import the library to read csv

import sys

# import the library to compute transformations from quaternion to euler
import tf
from tf.transformations import euler_from_quaternion

#ROS messages
#import appropiate ROS messages

from geometry_msgs.msg import Twist # For publish velocity
from nav_msgs.msg import Odometry # For subscribe the pose of turtlebot
class driver(object):
    
    def __init__(self):
        '''
        constructor
        '''
        #Initialize ros node
        rospy.init_node('turtlebot_driver')       

        #Initialize goals
        self.x = np.array([])
        self.y = np.array([])
        self.theta = np.array([])
        #Threshold for distance to goal
        self.goal_th_xy = rospy.get_param('goal_thershold_xy',0.1) #Position threshold
        self.goal_th_ang = rospy.get_param('goal_threshold_ang',0.2) #Orientation threshold
        
        #Point to the first goal
        self.active_goal = 0
        
        #Define subscriber
        rospy.Subscriber("/odom", Odometry, self.callback)
        
        #Define publisher        
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 20)
        
        #Define the velocity message
        self.vmsg = Twist();    
        
        #Has the goal been loaded?
        self.params_loaded = False            

    def print_goals(self):
        '''
        print_goals prints the list of goals
        '''
        rospy.loginfo("List of goals:")
        rospy.loginfo("X:\t " + str(self.x))
        rospy.loginfo("Y:\t " + str(self.y))
        rospy.loginfo("Theta:\t " + str(self.theta))
    
    def print_goal(self):
        '''
        pritn_goal prints the next goal
        '''
        rospy.loginfo( "Goal: (" + str(self.x[self.active_goal]) + " , " + str(self.y[self.active_goal]) + " , " + str(self.theta[self.active_goal]) + ")")
        
    def print_pose(self):
        '''
        print_pose pints the robot's actuall position
        '''
        rospy.loginfo( "Pose: (" + str(self.position_x) + " , " + str(self.position_y) + " , " + str(self.position_theta) + " )")
        
    def print_goal_and_pose(self):
        '''
        pritn_goal_and_pose prints both goal and pose
        '''
        rospy.loginfo("\tPose\t\tGoal")
        rospy.loginfo("X:\t%f\t%f",self.position_x,self.x[self.active_goal])
        rospy.loginfo("Y:\t%f\t%f",self.position_y,self.y[self.active_goal])
        rospy.loginfo("A:\t%f\t%f",self.position_theta,self.theta[self.active_goal])
        
    def dist_to_goal_xy(self):
        '''
        dist_to_goal_xy computes the distance in x and y direction to the 
        active goal
        '''
        return math.sqrt(pow(self.position_x-self.x[self.active_goal],2)+pow(self.position_y-self.y[self.active_goal],2))
    
    def dist_to_goal_ang(self):
        '''
        dist_to_goal_ang computes the orientation distance to the active
        goal
        '''
        return np.abs(angle_wrap(self.theta[self.active_goal]-self.position_theta[2]))
        
    def has_arrived_xy(self):
        '''
        has_arrived_xy returns true if the xy distance to the ative goal is
        smaller than the position threshold
        '''
        return self.dist_to_goal_xy()<self.goal_th_xy
        
    def has_arrived_ang(self):
        '''
        has_arrived_ang returns true if the orientation distance to the 
        ative goal is smaller than the orientation threshold
        '''
        return self.dist_to_goal_ang()<self.goal_th_ang
        
    def has_arrived(self):
        '''
        has_arrived return true if the robot is closer than the apropiate 
        threshold to the goal in position and orientation
        '''
        return (self.has_arrived_xy() and self.has_arrived_ang())   
    
    def check_goal(self):
        '''
        check_goal checks if the robot has arrived to the active goal, 
        '''
        if self.has_arrived():
            self.next_goal()
        
    def publish(self):
        '''
        publish publish the velocity message in vmsg
        '''
        self.pub.publish(self.vmsg)
        
    def callback(self,msg):
        '''
        callback reads the actuall position of the robot, computes the 
        appropiate velocity, publishes it and check if the goal is reached
        '''
        self.read_position(msg)
        self.compute_velocity()
        self.publish()
        self.check_goal()
        
    def drive(self):
        '''
        drive is a neede function for the ros to run untill somebody stops
        the driver
        '''
        self.print_goal()
        while not rospy.is_shutdown():
            rospy.sleep(0.03)
        
    def load_goals(self):
        '''
        load_goals loads the goal (or goal list for the option al part) into
        the x y theta variables.

        '''
        
        #f = open('/home/songyou/catkin_ws/src/probabilistic_labs/lab1_turtlebot/src/list.txt','r')
        filename = rospy.get_param('fname','')
        if filename != '':
            f = open(filename,'r')
            a,b,c = np.loadtxt(f,delimiter = ',', usecols=(0,1,2), unpack = True)
            self.x = np.append( self.x,a)
            self.y = np.append( self.y,b)
            self.theta = np.append( self.theta,c)
        else:
            self.x = np.append( self.x,rospy.get_param('x',0))
            self.y = np.append( self.y,rospy.get_param('y',0))
            self.theta = np.append( self.theta,rospy.get_param('theta',0))
        
        self.params_loaded = True
        self.print_goals()

        
    def next_goal(self):
        '''
        next_goal increments the index of the goal in 1 and checks whether
        or not the robot has reached the final goal
       
        '''
        
        if self.active_goal < self.x.size-1:
            self.active_goal = self.active_goal+1
        else:
            rospy.signal_shutdown('Final goal reached!')
        
    def read_position(self,msg):
        '''
        read_position copy the position received in the message (msg) to the
        internal class variables self.position_x, self.position_y and 
        self.position_theta
        
       
        '''

        self.position_x = msg.pose.pose.position.x # current pose in x direction
        self.position_y = msg.pose.pose.position.y # current pose in y direction
        quaternion = (msg.pose.pose.orientation.x,
        	msg.pose.pose.orientation.y,
        	msg.pose.pose.orientation.z,
        	msg.pose.pose.orientation.w)# current orienctation in quaternion form
        self.position_theta = euler_from_quaternion(quaternion)# Transform from quaternion to euler
        print ("position_x: %f" %self.position_x)
        print ("position_y: %f" %self.position_y)
    def compute_velocity(self):
        '''
        compute_velocity computes the velocity which will be published.
       
        '''
        self.vmsg=Twist()
        #Compute the angle from turtlebot to the goal
        angle_to_goal = math.atan2(self.y[self.active_goal]-self.position_y,self.x[self.active_goal]-self.position_x)
        #Compute the angle difference between turtlebot orientation and the angle from turtlebot to the goal
        ang_dif = angle_wrap(self.position_theta[2] - angle_to_goal)

        #If turtlebot has not reach the goal position
        if not self.has_arrived_xy():
            self.vmsg.angular.z = -0.2*ang_dif
            dis = math.sqrt(math.pow(self.y[self.active_goal] - self.position_y,2) + math.pow(self.x[self.active_goal] - self.position_x,2))
            if dis > 0.5:
                self.vmsg.linear.x =  0.1*dis
            else:
                self.vmsg.linear.x = 0.1
            print ("distance to current goal:%f" %dis)
        else:
                self.vmsg.angular.z = 0.6 # When arriving the position, rotate in order to meet the angle restriction     
                print ("rotating")