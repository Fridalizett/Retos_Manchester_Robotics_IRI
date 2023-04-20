#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32
from mini_c1.msg import signal_input
from geometry_msgs.msg import Twist
from math import pi
#Este es solo para ver que tipo de ruta va a seguir nuestro robot si el zig zag o el cuadrado
tipo=0
class square:
    def __init__(self):
        #variables para la velocidad de las llantas
        self.wr = 0.0
        self.wl = 0.0
        #Setup ROS subscribers and publishers
        #nos suscribimos a las llantas y lo guardamos en un mensaje
        rospy.Subscriber('/wr',Float32,self.wr_callback)
        rospy.Subscriber('/wl',Float32,self.wl_callback)
        rospy.Subscriber("set_point", signal_input,self.callback)
        #publicamos en la velocidad
        self.w_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        
        #inicializamos el nodo
        rospy.init_node("Control")
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.stop)

    # Callbacks for wheel velocities and commands
    def callback(self,msg):
        self.signal_data = msg.signal_y
        self.time_data = msg.time_x
    def wr_callback(self,msg):
        self.wr = msg.data

    def wl_callback(self,msg):
        self.wl = msg.data

    # Main function
    def run(self):
        # Variable initializations
        distance = 0.0
        angle = 0.0
        current_time = rospy.get_time()
        last_time = rospy.get_time()
        state = 0  # added state variable to keep track of movement
        state2 = 0
        # Create message for publishing
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        # Main Loop
        while not rospy.is_shutdown():
            # Compute time since last loop
            current_time = rospy.get_time()
            dt = current_time - last_time
            last_time = current_time
            # Update distance and angle from the velocity measurements
            distance += 0.05 * (self.wr + self.wl) * 0.5 * dt
            angle += 0.05 * (self.wr - self.wl) / 0.18 * dt
            self.wr = 0
            self.wl = 0
            #mandamos a llamr al parametro de lista 
            listax=rospy.get_param("list_of_x",[0.0, 0.9, 0.0, 1.124, 1.124])
            listay=rospy.get_param("list_of_y",[0.0, 0.5, 1.0, 1.734, 0])
            #colocamos en los puntos el valor de las listas que representaran x,y
            p0=np.array([listax[0],listay[0]]) 
            p1=np.array([listax[1],listay[1]])
            p2=np.array([listax[2],listay[2]])
            p3=np.array([listax[3],listay[3]])
            p4=np.array([listax[4],listay[4]])
            distancia=0

            #Movimiento del cuaddrado por el momento
                # States:
                # 0 = moving forward
                # 1 = turning right
                # 2 = moving backwards
                # 3 = turning left
            tipo=rospy.get_param("tipo",1)
            if tipo==1:
                if state == 0:
                    # moving forward
                    msg.angular.z = 0.0
                    msg.linear.x = self.signal_data#mandamos a llamar el mensaje de la senal 
                    if distance > 1:
                        # end of side reached
                        angle = 0.0
                        distance = 0.0
                        state = 1 
                elif state == 1:
                    # turning 
                    msg.linear.x = 0.0
                    msg.angular.z = pi/8 # turn 90 degrees
                    if angle > pi/3:
                        # end of turn reached
                        angle = 0.0
                        distance = 0.0
                        state = 0  
                # Publish message and sleep
            if tipo==2:
                if state2 == 0:
                    # turning right
                    msg.linear.x = 0.0
                    msg.angular.z = pi/8 
                    #tal vez lo use para despues medir los puntos dado cierta identidad trigonometrica
                    #angulo_f=np.arctan2(p1[1] - p0[1], p1[0] - p0[0])
                if angle > 0.1:
                    angle = 0.0
                    distance = 0.0
                    state2 = 1  
                elif state2 == 1:
                    # moving forward
                    msg.angular.z = 0.0
                    msg.linear.x = self.signal_data
                    distancia = np.linalg.norm(p1 - p0)
                    if distance > distancia:
                    # end of side reached
                        angle = 0.0
                        distance = 0.0
                        state2 = 2  
                elif state2 == 2:
                    msg.linear.x = 0.0
                    msg.angular.z = pi/8 
                    if angle > (2*pi)/5:
                        # end of side reached
                        angle = 0.0
                        distance = 0.0
                        state2 = 3  # start moving backwards
                elif state2 == 3:
                    msg.angular.z = 0.0
                    msg.linear.x = self.signal_data
                    distancia = np.linalg.norm(p2 - p1)
                    if distance > distancia:
                        # end of side reached
                        angle = 0.0
                        distance = 0.0
                        state2 = 4   
                elif state2 == 4:
                    msg.linear.x = 0.0
                    msg.angular.z = -pi/8 
                    if angle < -(3*pi)/5:
                        # end of turn reached
                        angle = 0.0
                        distance = 0.0
                        state2 = 5  # start moving backwards
                elif state2 == 5:
                    msg.angular.z = 0.0
                    msg.linear.x = self.signal_data
                    distancia = np.linalg.norm(p3 - p2)
                    if distance > distancia:
                        # end of side reached
                        angle = 0.0
                        distance = 0.0
                        state2=6
                elif state2 == 6:
                    # turning right
                    msg.linear.x = 0.0
                    msg.angular.z = -pi/8 
                    if angle < -(2*pi)/5:
                        # end of turn reached
                        angle = 0.0
                        distance = 0.0
                        state2 = 7 
                elif state2 == 7:
                    # moving forward
                    msg.angular.z = 0.0
                    msg.linear.x = self.signal_data
                    distancia = np.linalg.norm(p4 - p3)
                    if distance > distancia:
                        # end of side reached
                        t=0
                        angle = 0.0
                        distance = 0.0
                        state2=8
                #el auto deja de moverse 
                elif state2 == 8:
                    msg.angular.z = 0.0
                    msg.linear.x = 0.0
            rospy.loginfo(state2)
            self.w_pub.publish(msg)
            self.rate.sleep()

            # Publish message and sleep
            self.w_pub.publish(msg)
            self.rate.sleep()
 
    # Separate stop function for stopping when ROS shuts down
    def stop(self):
        print("Stopping")
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.w_pub.publish(msg)

if __name__ == "__main__":
    sq = square()

    try:
        sq.run()
    except rospy.ROSInterruptException:
        None
