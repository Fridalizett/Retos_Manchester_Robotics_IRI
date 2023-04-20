#!/usr/bin/env python
import rospy 
import numpy as np
from mini_c1.msg import signal_input
from std_msgs.msg import Float32

#inicializa el archivo set_point
msg = signal_input()
msg.time_x = 0.0
msg.signal_y = 0.0

#Creamos la funcion stop para cuando paremos el codigo
def stop(self):
    pub_signal.publish(0)
    rate.sleep()
    print("stopping")

#Empezamos a estructurar nuestro nodo 
if __name__=='__main__':
    rospy.init_node("Input")
    #Revisar si tenemos que modificar este rate? es el rate de transmicion me imagino
    rate= rospy.Rate(10)
    #Cuando se apaga encendemos la funcion stop
    rospy.on_shutdown(stop)
    #Publicamos el topic de nuestro nodo
    pub_signal = rospy.Publisher("set_point", signal_input,queue_size=10)
    print("The set point generator is running")
    t0= rospy.Time.now().to_sec()
    start_time=0
    #Aqui mandaremos una velocidad constante 
    while not rospy.is_shutdown():
        #estructura del seno 
        t= rospy.Time.now().to_sec()-t0
        timeset=t
        start_time+=0.1
        #estructura del square por lo tanto mandamos
        #velocidad constante atraves del tiempo 
        setout=rospy.get_param("velocidad",0.25)
        timeset=t
        msg.time_x = timeset
        msg.signal_y = setout
        #-------
        pub_signal.publish(msg)
        rospy.loginfo(timeset)
        rospy.loginfo(start_time)
        
        rate.sleep()              
    