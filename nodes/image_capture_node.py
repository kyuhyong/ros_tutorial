#!/usr/bin/env python
# license removed for brevity

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

class ImageCaptureNode:
    def __init__(self):
        self.msg_count = 0
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        # Node cycle rate (in Hz).
        #self.loop_rate = rospy.Rate(0.1)
        self.cv_image = None
        self.br = CvBridge()
        cv2.startWindowThread()
        cv2.namedWindow("Image window")
        rospy.loginfo("Starting: image capture node")
        #self.pub_stb = rospy.Subscriber('msg_tx', String, self.cb_subMsg, queue_size=10)
        # Subscribers
        rospy.Subscriber("/usb_cam/image_raw",Image, self.cb_image)
    
    def cb_image(self, msg):
        #rospy.loginfo("New image:")
        self.cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Image window", self.cv_image)
        self.msg_count+=1


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
    
    def main(self, fname):
        #rospy.spin()
        br = CvBridge()
        save_count = 0
        while not rospy.is_shutdown():
            key = self.getKey()
            if key == 'c':
                fileName = fname+'_'+str(save_count)+'.jpg'
                rospy.loginfo("Save image as "+fileName)
                cv2.imwrite(fname+'/'+fileName, self.cv_image)
                save_count+=1
            elif key == 'q':
                rospy.signal_shutdown("User Quit")
        
        exit()
            #self.loop_rate.sleep()

if __name__ == '__main__':
    msg = 'ARG0: {}'.format(sys.argv[1])
    print(msg)
    fname = None
    if len(sys.argv)>1:
        fname = sys.argv[1]
        if not os.path.exists(fname):
            os.makedirs(fname)
    try :
        rospy.init_node('image_capture_node')
        node = ImageCaptureNode()
        node.main(fname)
    except rospy.ROSInterruptException:
        pass