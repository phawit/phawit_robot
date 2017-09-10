#!/usr/bin/env python

import rospy
import actionlib
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalID

PString = rospy.Publisher('say', String, queue_size=10)
PCmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
PGoal = rospy.Publisher('move_base/goal', MoveBaseGoal, queue_size=10)
PCancle = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
def callback(data):
 if data.data == "stop":
    stop()
 elif data.data == "turn right":
    turn_right()
 elif data.data == "turn left":
    turn_left()
 elif data.data == "go to point 1":
    point_1()
 elif data.data == "go to point 2":
    point_2()
 elif data.data == "go to home":
    home()
 elif data.data == "cancle":
    cancle()
 else:    
    rospy.loginfo("string")
    say = String()
    PString.publish("talk")
    
    rospy.loginfo("cmd")
    cmd = Twist()
    cmd.linear.x = 0; cmd.linear.y = 0; cmd.linear.z = 0
    cmd.angular.x = 0; cmd.angular.y = 0; cmd.angular.z = 0
    PCmd.publish(cmd)

    rospy.loginfo("movebase_goal")
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x=1.1
    goal.target_pose.pose.position.y=2.2
    goal.target_pose.pose.position.z=2.3
    goal.target_pose.pose.orientation.x = 5.3
    goal.target_pose.pose.orientation.y= 3.2
    goal.target_pose.pose.orientation.z= 2.3
    goal.target_pose.header.frame_id= 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    PGoal.publish(goal)

    #cancel_all_goals()
    rospy.loginfo("movebase_cancle")
    cancel = GoalID()
    PCancle.publish(cancel)
    
def stop():
    rospy.loginfo("stop")
    cmd = Twist()
    cmd.linear.x = 0; cmd.linear.y = 0; cmd.linear.z = 0
    cmd.angular.x = 0; cmd.angular.y = 0; cmd.angular.z = 0
    PCmd.publish(cmd)

def turn_right():
    rospy.loginfo("turn_right")

def turn_left():
    rospy.loginfo("turn_left")

def point_1():
    rospy.loginfo("point_1")

def point_2():
    rospy.loginfo("point_2")

def home():
    rospy.loginfo("home")

def cancle():
    rospy.loginfo("cancle")

def setup():
    rospy.init_node('action', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.loginfo("hello")
    rospy.spin()
        
if __name__ == '__main__':
    setup()
    
