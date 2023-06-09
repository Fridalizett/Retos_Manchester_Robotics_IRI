#!/usr/bin/env python

import rospy
import numpy as np
import math
import time
from std_msgs.msg import Float32
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
#from puzzlebot_reto1.msg import set_point
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge

import cv2 

image = None

redBajo2=np.array([160, 100, 20], np.uint8)
redAlto2=np.array([179, 250, 255], np.uint8)

greenBajo1 = np.array([55, 100, 20], np.uint8)
greenAlto1 = np.array([90, 250, 255], np.uint8)

yellowBajo1 = np.array([20, 100, 20], np.uint8)
yellowAlto1 = np.array([30, 250, 255], np.uint8)

font = cv2.FONT_HERSHEY_SIMPLEX

a=0
a2=0
k=0
r=1
z=0
h=0
q=0
n=0
o=0
w2=0
x_min=0

l = 0.19
m = 0.05


ultima_medicion_a = 0.0
ultima_medicion_d = 0.0

error_acumulado_a = 0.0
error_acumulado_d = 0.0

error_sum_a = 0.0
error_sum_d = 0.0

first = True

pose = Pose2D()
pose.x = 0.0
pose.y = 0.0
pose.theta = 0.0

current_time = 0.0
previous_time = 0.0
ang=0
ang2=0

roi_upper = 0.70
roi_lower = 0.99


frameWidth = 640
frameHeight = 480

setpoint=0
error_a=0
#error_d =1

th1=0
global i,j
i=0
j=0


def dibujar(mask,color,num):
  contornos,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  for c in contornos:
    area = cv2.contourArea(c)
    if area > 3000:
      M = cv2.moments(c)
      if (M["m00"]==0): M["m00"]=1
      x = int(M["m10"]/M["m00"])
      y = int(M['m01']/M['m00'])
      nuevoContorno = cv2.convexHull(c)
      cv2.circle(image,(x,y),7,(0,255,0),-1)
      cv2.putText(image,'{},{}'.format(x,y),(x+10,y), font, 0.75,(0,255,0),1,cv2.LINE_AA)
      cv2.drawContours(image, [nuevoContorno], 0, color, 3)

      epsilon = 0.01*cv2.arcLength(c,True)
      approx = cv2.approxPolyDP(c,epsilon,True)
      x,y,w,h = cv2.boundingRect(approx)
      if len(approx)>4:
          cv2.putText(image,'Circulo', (x,y-5),1,1.5,(0,255,0),2)
          if num == 1:
              print("circulo rojo")
              global w2
	      w2=2
	                   
          elif num == 2:
              print("circulo amarillo")
              w2=1
              

          elif num == 3:
	      w2=0
              print("circulo verde")

                             
	          #stop()



def callback(msg):
    global vel,z,a,x_deseada
    x_i=msg.data
    x_deseada = x_i.split(delim)
    z=1
    a=a+1

def callback2(msg):
    global ang,k,y_deseada
    y_i=msg.data
    y_deseada = y_i.split(delim)
    k=1

def imag_callback(data):
    global image ,a2,br
    br = CvBridge()  
    ima = br.imgmsg_to_cv2(data)
    image = cv2.rotate(ima,cv2.ROTATE_180)
    
    
    a2=1

def wr_callback(msg):
    global wr
    wr = msg.data

def wl_callback(msg):
    global wl
    wl = msg.data

def stop(self):
    print("Stopping")
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0
    w_pub.publish(msg)


def wrap_to_pi(theta):
    result=np.fmod((theta+ np.pi),(2*np.pi))
    if(result<0):
	result += 2 * np.pi
    return result - np.pi

def seg_linea(image):
    global ang2 ,x_min ,setpoint,error_a
    gray= cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    th=cv2.inRange(image,(0,0,0),(90,90,90))
    bordes = cv2.Canny(th , 100,200)
    cnts, _= cv2.findContours(th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts=sorted(cnts, key=cv2.contourArea,reverse=True)[:1]
    if len(cnts) > 0:
        x,y,w,h = cv2.boundingRect(cnts[0])
        blackbox = cv2.minAreaRect(cnts[0])
        (x_min,y_min), (w_min, h_min),ang2 = blackbox
    cv2.drawContours(image, cnts,0,(0,255,0),3)
    h, w, _ =image.shape
    setpoint = w/2
    cv2.line(image, (int(w/2),10),(int(w/2),40),(255,0,0),3)
    lines1 =cv2.HoughLines(bordes,1,np.pi/180,200)


def fit_line_and_calculate_error(angles):
        global error_a
        histogram, _ = np.histogram(angles, bins=180, range=(-90, 90))
        error_a = histogram.argmax() - 90

def senales(image):
    image2 = cv2.resize(image,(frameWidth, frameHeight)) 
    

def imag():
    image2 = cv2.resize(image,(frameWidth, frameHeight)) 
    frameHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    roi_size = [int(roi_upper*frameHeight),int(roi_lower*frameHeight),40,frameWidth - 50]
    lineThresh = 0.1
    roi = image2[roi_size[0]:roi_size[1],roi_size[2]:roi_size[3]]
    seg_linea(roi)
    senales(image)

    maskRed = cv2.inRange(frameHSV, redBajo2, redAlto2)
    maskRedvis = cv2.bitwise_and(image, image, mask= maskRed)
    maskyellow = cv2.inRange(frameHSV, yellowBajo1, yellowAlto1)
    maskgreen = cv2.inRange(frameHSV, greenBajo1, greenAlto1)
	    
    dibujar(maskyellow,(0,255,255),2)
    dibujar(maskRed,(0,0,255),1)
    dibujar(maskgreen,(0,255,0),3)
    cv2.imshow("lineal",roi)
    cv2.imshow("principal",image2)
    cv2.waitKey(1)

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")
    
    wr = 0.0
    wl = 0.0
    rate = rospy.Rate(100)
    delim = ','
    sub = rospy.Subscriber('/help_msg', String, callback)
    sub2 = rospy.Subscriber('/help_msg2', String, callback2)
    rospy.Subscriber('video_frames', Image, imag_callback)

#volores de nuestro controlador
    kp_a=0.01
    kp_d=0.01

    ki_a=0.0
    ki_d=0.00

    kd_a= 0.00
    kd_d= 0.0

    # Create message for publishing
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0

    #Variable initialisation
    current_time = rospy.get_time()
    last_time = rospy.get_time()
    while not rospy.is_shutdown():
            
	if z==1 and k==1 and a2==1:
	    if r==1 :	

		x_i3=len(x_deseada) 
	        y_i3=len(y_deseada)

	        current_time = rospy.get_time() 
                previous_time = rospy.get_time()
		print(x_i3)
		
	        r=0

	    # Compute time since last loop
	    rospy.Subscriber('/wr',Float32,wr_callback)
            rospy.Subscriber('/wl',Float32,wl_callback)
            imag()
           

	    w_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
            pose_pub = rospy.Publisher("/pose",Pose2D,queue_size=1)
            loop_rate = rospy.Rate(10)


            current_time = rospy.get_time()
            dt = (current_time - previous_time)
            previous_time = current_time
	    
	    
	    pose.theta = wrap_to_pi(pose.theta + dt * m * ((wr - wl) / l))
            pose.x += dt * m * ((wr + wl) / 2) * np.cos(pose.theta)
            pose.y += dt * m * ((wr + wl) / 2) * np.sin(pose.theta)

  	    error_a = int(setpoint - x_min)
        
  	    if error_a < 18 and error_a > -12:
    		error_a=0
            error_d = error_a
 
            #errores angulares
	    error_sum_a += error_a * dt
            error_derivativo_a= (error_a-  error_acumulado_a) / (dt+0.00001)
            error_acumulado_a = error_a
            #controlador angular
            accion_proporcional_a = kp_a * error_a
            accion_integral_a = ki_a * error_sum_a
            accion_deribativa_a =kd_a * error_derivativo_a
            control_a= (accion_proporcional_a + accion_integral_a + accion_deribativa_a)
            
            #errores lineales
            error_sum_d += error_d * dt
            error_derivativo_d= (error_d-  error_acumulado_d) / (dt+0.00001)
            #error_acumulado_d = error_d
            #controlador lineal
            accion_proporcional_d = kp_d * error_d 
            accion_integral_d = ki_d * error_sum_d
     
            control_d= (accion_proporcional_d + accion_integral_d )

	    #control_d= (accion_proporcional_d + accion_integral_d)
            control_d= control_d/8
            control_a= control_a/10

            if control_d >0.95:
                control_d = control_d/2
            control_d= abs(control_d)
           
            if control_a >0.95 :
                control_a = control_a/2
     

            if w2 == 0:
                if error_d ==0: 
                    msg.linear.x = 0.05
                else:
                    msg.linear.x = control_d/2
                msg.angular.z = control_a/2
            elif w2 == 1:
	        if error_d ==0: 
                    msg.linear.x = 0.05
                else:
                    msg.linear.x = control_d/3
                msg.angular.z = control_a/3
	    elif w2 == 2:
                msg.linear.x = 0.0
                msg.angular.z = 0.0


	    print_info = "%3f | %3f |     %3f | %3f " %(error_d,control_d , error_a,control_a)
            rospy.loginfo(print_info)


            w_pub.publish(msg)
	    pose_pub.publish(pose)
            rate.sleep()  
