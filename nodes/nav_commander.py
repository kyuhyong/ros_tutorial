#!/usr/bin/env python

import sys
import rospy
import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class NavCommanderNode:
    def __init__(self):
        rospy.init_node("nav_commander")
        self.pub_init_pose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.header.frame_id = "map"
        self.init_pose.header.stamp = rospy.Time.now()
        self.init_pose.pose.pose.position.x = 0.0
        self.init_pose.pose.pose.position.y = 0.0
        self.init_pose.pose.pose.orientation.w = 1.0

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"

        self.update_init_pose(0.0, 0.0, -0.2)
        self.pub_init_pose.publish(self.init_pose)
        self.send_goal(1.5,1.5,1.57)
        wait = self.client.wait_for_result()
        if not wait:
            print('Error')
        else:
            print(self.client.get_result())

        self.send_goal(0.0,0.0,0.0)
        wait = self.client.wait_for_result()
        if not wait:
            print('Error')
        else:
            print(self.client.get_result())

    
    def update_init_pose(self, x, y, theta):
        self.init_pose.header.stamp = rospy.Time.now()
        self.init_pose.pose.pose.position.x = x
        self.init_pose.pose.pose.position.y = y
        self.init_pose.pose.pose.orientation.w = 1.0
        q = quaternion_from_euler(0.0, 0.0, theta)
        self.init_pose.pose.pose.orientation.x = q[0]
        self.init_pose.pose.pose.orientation.y = q[1]
        self.init_pose.pose.pose.orientation.z = q[2]
        self.init_pose.pose.pose.orientation.w = q[3]

    def send_goal(self,x,y,theta):
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, theta)
        self.goal.target_pose.pose.orientation.x=q[0]
        self.goal.target_pose.pose.orientation.y=q[1]
        self.goal.target_pose.pose.orientation.z=q[2]
        self.goal.target_pose.pose.orientation.w=q[3]
        self.client.send_goal(self.goal)
    
    def main(self):
        rospy.spin()

if __name__== '__main__':
    try:
        rospy.init_node('nav_commander')
        node = NavCommanderNode()
        node.main()
    except rospy.ROSInterruptException:
        pass