#!/usr/bin/env python
import roslib; roslib.load_manifest('lab0_ros')
import rospy
from probabilistic_lib import functions

#For command line arguments
import sys
#For atan2
import math

#TODO: Import the messages we need
##
##
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

#Initialization of turtle position
x=None
y=None
theta=None

#Position tolerance for both x and y
tolerance=0.1
#Have we received any pose msg yet?
gotPosition=False

def callback(pose_msg):
    global x,y,theta,gotPosition
    #TODO:Store the position in x,y and theta variables.
    x = pose_msg.x
    y = pose_msg.y
    theta = pose_msg.theta
    gotPosition=True
    #rospy.loginfo('here')

def waypoint():
    global gotPosition
    vel_msg = Twist()
    #TODO: Define the pulisher: Name of the topic. Type of message
    #
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 20)
    
    #Name of the node
    rospy.init_node('turtle_waypoint')
    #TODO: Define the subscriber: Name of the topic. Type of message. Callback function
    rospy.Subscriber("/turtle1/pose", Pose, callback)
    #Has the turtle reach position?
    finished=False
    #If the point hasn't been specified in a command line:
    if(len(sys.argv)!=3):
        print('X and Y values not set or not passed correctly. Looking for default parameters.')
        #TODO: If ROS parameters default_x and default_y exist:
        if rospy.has_param('/default_x') and rospy.has_param('/default_y'): #Change this for the correct expression
            #TODO: Save them into this variables
            x_desired=rospy.get_param('/default_x') #Change this for the correct expression
            y_desired=rospy.get_param('/default_y') #Change this for the correct expression
            print('Heading to: %f,%f' %(x_desired, y_desired))
        else:
            print('Default values parameters not set!. Not moving at all')
            finished=True
    else:
        #Save the command line arguments.
        x_desired=float(sys.argv[1])
        y_desired=float(sys.argv[2])
        print('Heading to: %f,%f' %(x_desired, y_desired))

    while not rospy.is_shutdown() and not finished:
        if(gotPosition):
            #TODO: Send a velocity command for every loop until the position is reached within the tolerance.            
            
            # turn
            angle_to_goal = math.atan2(y_desired-y,x_desired-x)
            ang_dif = functions.angle_wrap(theta - angle_to_goal)
            vel_msg.angular.z = -15 * ang_dif
 
            # go straight
            dis = math.sqrt(math.pow(y_desired - y,2) + math.pow(x_desired - x,2))
            vel_msg.linear.x =  5 * dis
            
            # publish
            pub.publish(vel_msg)
            
            if abs(x-x_desired)<=tolerance and abs(y-y_desired)<=tolerance:
                print('x: %f' %x)
                print('y: %f' %y)
                print('Arrive!!')
                vel_msg.angular.z = 0
                vel_msg.linear.x = 0
                finished=True 


        #Publish velocity commands every 0.3 sec.
        rospy.sleep(0.05)

if __name__ == '__main__':
    try:
        waypoint()
    except rospy.ROSInterruptException:
        pass
