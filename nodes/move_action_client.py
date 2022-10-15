#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

from tf.transformations import quaternion_from_euler, euler_from_quaternion

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

class ActionClientNode:
    def __init__(self):
        self.count = 0
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        rospy.loginfo("Starting: message publisher node")
        self.pub_init = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.header.frame_id = "map"
        self.init_pose.header.stamp = rospy.Time.now()
        self.init_pose.pose.pose.position.x = 0.0
        self.init_pose.pose.pose.position.y = 0.0
        self.init_pose.pose.pose.orientation.w = 1.0
        self.init_theta = 0.0
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()


    def getKey(self):
        if os.name == 'nt':
            return msvcrt.getch()

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def send_goal(self, goal):
        # Send the goal to the action server.
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logger("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def update_init_pose(self, x, y, theta):
        self.init_pose.header.stamp = rospy.Time.now()
        self.init_pose.pose.pose.position.x += x
        self.init_pose.pose.pose.position.y += y
        self.init_theta += theta
        q = quaternion_from_euler(0.0, 0.0, self.init_theta)
        rospy.loginfo(self.init_pose.pose.pose)
        self.init_pose.pose.pose.orientation.x = q[0]
        self.init_pose.pose.pose.orientation.y = q[1]
        self.init_pose.pose.pose.orientation.z = q[2]
        self.init_pose.pose.pose.orientation.w = q[3]


    def main(self):
        #rospy.spin()
        while(1):
            key = self.getKey()
            if key == 'g':
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                # Move 0.5 meter forward along x axis of the "map" coordinate
                goal.target_pose.pose.position.x = 0.5
                # No rotation of base frame 
                goal.target_pose.pose.orientation.w = 1.0
                rospy.loginfo(self.send_goal(goal))
            
            elif key == 'i':
                self.update_init_pose(0.1, 0.0, 0.0)
                self.pub_init.publish(self.init_pose)
            elif key == 'k':
                self.update_init_pose(-0.1, 0.0, 0.0)
                self.pub_init.publish(self.init_pose)

            elif key == 'j':
                self.update_init_pose(0.0, 0.1, 0.0)
                self.pub_init.publish(self.init_pose)

            elif key == 'l':
                self.update_init_pose(0.0, -0.1, 0.0)
                self.pub_init.publish(self.init_pose)

            elif key == 'u':
                self.update_init_pose(0.0, 0.0, 0.1)
                self.pub_init.publish(self.init_pose)

            elif key == 'o':
                self.update_init_pose(0.0, 0.0, -0.1)
                self.pub_init.publish(self.init_pose)

            elif key == 'q':
                exit()

if __name__ == '__main__':
    try :
        rospy.init_node('action_client_node')
        node = ActionClientNode()
        node.main()
    except rospy.ROSInterruptException:
        pass