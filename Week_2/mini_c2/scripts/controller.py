#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
from mini_c2.msg import input_point

class Controller:
    def __init__(self):
        # Configurar parametros del controlador
        self.kp_l = 0.09
        self.ki_l = 0.0
        self.kd_l = 0.0
        self.kp_ang = 0.4
        self.ki_ang = 0.03
        self.kd_ang = 0.0


        self.setpoint = 0.0
        self.error_sum_l = 0.0
        self.error_prev_l = 0.0
        self.error_diff_l = 0.0
        self.error_sum_ang = 0.0
        self.error_prev_ang = 0.0
        self.error_diff_ang = 0.0
        self.error_dist=0.0
        self.error_ang=0.0
        self.first=True
        self.velocidad_l=0
        self.velocidad_ang=0
        self.controlador_vl=0.0
        self.controlador_va=0.0        
        self.last_vl=0.0
        self.last_va=0.0

        #Inicializar nodos
        rospy.init_node("controller")
        rospy.Subscriber("/wr",Float32,self.wr_callback)
        rospy.Subscriber("/wl",Float32,self.wl_callback)
        rospy.Subscriber("/error",input_point,self.callback)   
        self.pose_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        self.msg = Twist()
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0

        self.rate = rospy.Rate(10)

    # Callbacks para velocidades de las ruedas
    def callback(self, msg):
        self.error_dist=msg.ed
        self.error_ang=msg.eang

    def wr_callback(self, msg):
        self.wr = msg.data
    
    def wl_callback(self, msg):
        self.wl = msg.data

    def stop(self):
        print("Stopping")
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0
        self.pose_pub.publish(self.msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.first:
                self.current_time = rospy.get_time() 
                self.previous_time = rospy.get_time()
                self.first = False
            else:
                self.current_time = rospy.get_time() 
                dt = (self.current_time - self.previous_time)
                self.previous_time = self.current_time
                #Controlador lineal  
                self.error_sum_l += self.error_dist * dt
                self.error_diff_l = 0#(self.error_dist - self.error_prev_l) / dt
                self.error_prev_l = self.error_dist
                self.controlador_vl= self.kp_l * self.error_dist + self.ki_l * self.error_sum_l + self.kd_l * self.error_diff_l
                self.velocidad_l =self.last_vl +((self.controlador_vl-self.last_vl))
                self.last_vl=self.velocidad_l
                #controlador angular
                self.error_sum_ang += self.error_ang * dt
                self.error_diff_ang = 0#(self.error_ang - self.error_prev_ang) / dt
                self.error_prev_ang = self.error_ang
                self.controlador_va =self.kp_ang * self.error_ang + self.ki_ang * self.error_sum_ang + self.kd_ang * self.error_diff_ang
                self.velocidad_ang = self.last_va +((self.controlador_va-self.last_va))
                self.last_va=self.velocidad_ang
             #Publicar las posiciones
                self.msg.linear.x = self.velocidad_l
                self.msg.angular.z = self.velocidad_ang
                print_info = "%3f | %3f  " %(self.velocidad_l,self.velocidad_ang)
                rospy.loginfo(print_info)
                self.pose_pub.publish(self.msg)
                self.rate.sleep()

if __name__ == "__main__":
    controller = Controller()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        None
    