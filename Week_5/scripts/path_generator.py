#!/usr/bin/env python
import rospy
import numpy as np
import std_msgs.msg
#from puzzlebot_reto1.msg import set_point
from std_msgs.msg import String


if __name__=='__main__':
    rospy.init_node("path_generator")
    x_i = rospy.get_param("x_i") 
    y_i = rospy.get_param("y_i")

    help_msg = String()
    help_msg2 = String()
    help_msg.data = x_i
    help_msg2.data = y_i
    pub = rospy.Publisher('/help_msg', String, queue_size = 1)
    pub2 = rospy.Publisher('/help_msg2', String, queue_size = 1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
	pub.publish(help_msg)
        pub2.publish(help_msg2)
        print(help_msg)
        print(help_msg2)
        rate.sleep()

	

		
