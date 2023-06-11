#!/usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist
#from puzzlebot_reto1.msg import set_point
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import matplotlib.animation as FuncAnimation
import cv2 

image = None

redBajo2=np.array([160, 100, 20], np.uint8)
redAlto2=np.array([179, 255, 255], np.uint8)

greenBajo1 = np.array([55, 100, 20], np.uint8)
greenAlto1 = np.array([100, 255, 255], np.uint8)

yellowBajo1 = np.array([15, 100, 20], np.uint8)
yellowAlto1 = np.array([45, 255, 255], np.uint8)


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


roi_upper = 0.60
roi_lower = 0.99

roi_upper2 = 0.60
roi_lower2 = 0.99

frameWidth = 640
frameHeight = 480

setpoint=0

global i,j
i=0
j=0

#no ocupamos esta funcion ahorita
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
	      w2=1 
	                   
          elif num == 2:
              print("circulo amarillo")
              

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
    image = br.imgmsg_to_cv2(data)
    
    
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

#en esta funcion estoy haciendo todo lo del seguidor de linea
def seg_linea(image,image2):
    global ang , setpoint 
    hist_n = np.sum(image,axis=0,dtype=np.float32)
    zero = np.argwhere(hist_n == 0)
    hist_n1 = np.delete(hist_n,zero)
    line=np.argmin(hist_n) - 320
    #lines=np.argmin(hist_n1) - 320
    #print(lines) 
    lineTresh = 0.1
    #sacamos los dos radientes
    gradiente1 = np.gradient(hist_n)
    gradiente2 = np.gradient(hist_n)
    #Aplicamos el umbral de ruido
    thresh_pos = np.average(gradiente1) + (lineTresh) * (gradiente1.max() - gradiente1.min())
    thresh_neg = np.average(gradiente1) - (lineTresh) * (gradiente1.max() - gradiente1.min())

    right_grad =(gradiente1 > thresh_pos) * gradiente2
    left_grad =(gradiente1 < thresh_pos) * gradiente2
    #eliminamos la parte negativa 
    left_grad[left_grad < 0] = 0
    right_grad[right_grad < 0] = 0
    #graficamos nuestros gradientes
    lines=np.argmin(hist_n1) 
    #lines=np.argmin(right_grad) 
    lines=abs(lines)

    print(lines)

    #plt.plot(left_grad)
    #plt.plot(right_grad)
    #plt.plot(th[20])
    plt.plot(hist_n)
    plt.pause(0.0005)
    plt.clf()

    an_vent = 25
    copy = image2.copy()

    x1 = int(lines + 280 + an_vent / 2)
    x2 = int(lines + 280 - an_vent / 2)
    y1 = int(roi_lower2)
    y2 = int(roi_upper2)


    x3 = int(lines  + an_vent / 2)
    x4 = int(lines  - an_vent / 2)
    y3 = int(roi_lower2)
    y4 = int(roi_upper2)

    x5 = int(lines - 280 + an_vent / 2)
    x6 = int(lines - 280 - an_vent / 2)
    y5 = int(roi_lower2)
    y6 = int(roi_upper2)

    cv2.rectangle(copy,(x1,y1),(x2,y2),(150,200,50),-1)
    cv2.rectangle(copy,(x3,y3),(x4,y4),(150,200,50),-1)
    cv2.rectangle(copy,(x5,y5),(x6,y6),(150,200,50),-1)
    #plt.subplot(111),plt.imshow(copy,cmap = 'gray'),plt.title('Original'), plt.axis('off')
    print("borde izquierdo : ",x3)
    print("borde derecho : ",x4)
    #cv2.imshow("bin",image)
    cv2.imshow("copy",copy)
    cv2.waitKey(1)



#esta funcion solo recorto la imagen y se la mando a la funcion de arriba
def imag():
    global roi_upper2,roi_lower2
    image2 = cv2.resize(image,(frameWidth, frameHeight))
    #image2 = cv2.cvtColor(image2,cv2.COLOR_BGR2RGB) 
 
    kernel = np.ones((5,5),np.float32)/20 
    #image= cv2.filter2D(image,-1,kernel)
    gray= cv2.cvtColor(image2,cv2.COLOR_BGR2GRAY)
    gray= cv2.GaussianBlur(gray,(21,21),0)
    retval, th = cv2.threshold(gray,100,255,0)

    roi_size = [int(roi_upper*frameHeight),int(roi_lower*frameHeight),0,frameWidth - 1]
    roi_upper2 = roi_size[0]
    roi_lower2 = roi_size[1]
    lineThresh = 0.1
    roi = th[roi_size[0]:roi_size[1],roi_size[2]:roi_size[3]]
    seg_linea(roi,image2)

    cv2.imshow("o2",roi)
    cv2.imshow("gray",gray)
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
    kp_a=1.0
    kp_d=0.05

    ki_a=0.02
    ki_d=0.02

    kd_a= 0.003

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
    # aqui abajo ahorita solo estamos llamando a las funciones para el audio qui mismo iria el controlador y todo eso
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

	    print_info = "%3f | %3f " %(ang,previous_time)
            rospy.loginfo(print_info)

            w_pub.publish(msg)
	    pose_pub.publish(pose)
            rate.sleep()           
