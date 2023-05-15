#!/usr/bin/env python

from ast import Pass
import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image

class procesamiento:
    def __init__(self):
        self.ancho = 640
        self.alto = 320
        
        self.image = None	
        self.bridge = cv_bridge.CvBridge()
        self.green_time = 0
        self.red_time = 0
        self.yellow_time = 0
	#Definir suscriptores y publicadores
        self.image_sub = rospy.Subscriber('/video_source/raw',Image, self.image_callback) 

        self.image_pub = rospy.Publisher('/procesada',Image, queue_size=10)

    def image_callback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    def preprocesamiento(self,image):
        rotar = cv2.rotate(image,cv2.ROTATE_180)
        redimensionar = cv2.resize(rotar,(self.ancho, self.alto))
        src_gray = cv2.cvtColor(redimensionar, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(src_gray, (19, 19), 0)
        return blurred
    def detect_traffic_light(self):
        frame = self.image  # Utilizar la imagen almacenada en self.image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                               param1=50, param2=30,
                               minRadius=50, maxRadius=65)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                color = frame[i[1], i[0]]
                if color[2] > 100 and color[1] < 100 and color[0] < 100:
                    cv2.circle(frame,(i[0],i[1]),i[2],(0,0,255),2)
                    cv2.putText(frame, 'rojo', (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), lineType=cv2.LINE_AA)
                    self.yellow_time = 0
                    self.green_time = 0
                    self.red_time += 1
                    if self.red_time > 2*30: # 2 segundos a una frecuencia de 30 fps
                        print("1")
                elif color[2] > 100 and color[1] > 100 and color[0] < 100:
                    cv2.circle(frame,(i[0],i[1]),i[2],(0,255,255),2)
                    cv2.putText(frame, 'amarillo', (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), lineType=cv2.LINE_AA)
                    self.yellow_time += 1
                    if self.yellow_time > 2*30: # 2 segundos a una frecuencia de 30 fps
                        print("2")
                        self.red_time = 0
                        self.green_time = 0
                elif color[1] > color[0] and color[1] > color[2] and color[0] < 100 and color[2] < 100:
                    cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
                    cv2.putText(frame, 'verde', (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv2.LINE_AA)
                    self.green_time += 1
                    if self.green_time > 2*30: # 2 segundos a una frecuencia de 30 fps
                        print("3")
                else:
                    pass
        cv2.imshow('Circulos detectados', frame)
        cv2.waitKey(1)

    def run(self):
        if self.image is None:
            return
        procesada = self.preprocesamiento(self.image)
        #detection=self.detect_traffic_light(self.image)
        ros_img = self.bridge.cv2_to_imgmsg(procesada)
        #ros_img = self.bridge.cv2_to_imgmsg(detection)

        self.image_pub.publish(ros_img)



if __name__ == "__main__":
    image_proc = procesamiento()
    rospy.init_node("procesamiento")
    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            image_proc.run()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass
