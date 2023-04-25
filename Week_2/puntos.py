#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

import numpy as np


 # Variables para el almacenamiento de las velocidades de las ruedas

wr = 0.0
wl = 0.0

first = True

 # Variables para el almacenamiento de las posiciones del robot
pose = Pose2D()
pose.x = 0.0
pose.y = 0.0
pose.theta = 0.0

current_time = 0.0
previous_time = 0.0

# Parametros fisicos del robot
l = 0.19
r = 0.05
		


 # Callbacks para velocidades de las ruedas
def wr_callback(msg):
        global wr
        wr = msg.data
    
def wl_callback(msg):
        global wl
        wl = msg.data

        
if __name__ == "__main__":
    #Inicializar nodos
    rospy.init_node("controller")
    
    rospy.Subscriber("/wr",Float32,wr_callback)
    rospy.Subscriber("/wl",Float32,wl_callback)

    pose_pub = rospy.Publisher("/pose",Pose2D,queue_size=1)

    #current_time = rospy.get_time()
    #previous_time = rospy.get_time()
    loop_rate = rospy.Rate(10)
    

    try:
        while not rospy.is_shutdown():
            # Compute time since last main loop
            if first == True:
                current_time = rospy.get_time() 
                previous_time = rospy.get_time()
                #current_time = rospy.get_time()
                first = False
            else:
                
            
                current_time = rospy.get_time()
                dt = (current_time - previous_time)
                previous_time = current_time

        # Actualizar las posiciones
                
                pose.theta = pose.theta + dt * r * ((wr - wl) / l)
                pose.x += dt * r * ((wr + wl) / 2) * np.cos(pose.theta)
                pose.y += dt * r * ((wr + wl) / 2) * np.sin(pose.theta)

        # Publicar las posiciones
                pose_pub.publish(pose)
                loop_rate.sleep()
    except rospy.ROSInterruptException:
        pass


    
