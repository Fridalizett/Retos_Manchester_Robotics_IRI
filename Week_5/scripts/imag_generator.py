#!/usr/bin/env python

import rospy 
import numpy as np
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import cv_bridge
image = None
a=0

def image_callback(msg):
    global image,a
    image = br.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    a=1

if __name__ == '__main__':
    pub = rospy.Publisher('video_frames', Image, queue_size=10)
    image_sub = rospy.Subscriber('/video_source/raw',Image, image_callback)
    rospy.init_node('imag_generator', anonymous=True)
    rate = rospy.Rate(10)      
    br =  cv_bridge.CvBridge()

    while not rospy.is_shutdown():
        if a == 1:
            rospy.loginfo('publishing video frame')
            ros_imag= br.cv2_to_imgmsg(image)         
            pub.publish(ros_imag)     
            rate.sleep()
         

