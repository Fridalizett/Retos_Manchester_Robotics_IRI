import numpy as np
import cv2 as cv

class TrafficLightDetector:
    def __init__(self, camera=0):
        self.cap = cv.VideoCapture(camera)
        self.green_time = 0
        self.red_time = 0
        self.yellow_time = 0
    
    def detect_traffic_light(self):
        ret, frame = self.cap.read()
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray = cv.medianBlur(gray, 5)
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, 20,
                               param1=50, param2=30,
                               minRadius=50, maxRadius=65)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                color = frame[i[1], i[0]]
                if color[2] > 100 and color[1] < 100 and color[0] < 100:
                    cv.circle(frame,(i[0],i[1]),i[2],(0,0,255),2)
                    cv.putText(frame, 'rojo', (i[0], i[1]), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), lineType=cv.LINE_AA)
                    self.yellow_time = 0
                    self.green_time = 0
                    self.red_time += 1
                    if self.red_time > 2*30: # 2 segundos a una frecuencia de 30 fps
                        print("1")
                elif color[2] > 100 and color[1] > 100 and color[0] < 100:
                    cv.circle(frame,(i[0],i[1]),i[2],(0,255,255),2)
                    cv.putText(frame, 'amarillo', (i[0], i[1]), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), lineType=cv.LINE_AA)
                    self.yellow_time += 1
                    if self.yellow_time > 2*30: # 2 segundos a una frecuencia de 30 fps
                        print("2")
                        self.red_time = 0
                        self.green_time = 0
                elif color[1] > color[0] and color[1] > color[2] and color[0] < 100 and color[2] < 100:
                    cv.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
                    cv.putText(frame, 'verde', (i[0], i[1]), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), lineType=cv.LINE_AA)
                    self.green_time += 1
                    if self.green_time > 2*30: # 2 segundos a una frecuencia de 30 fps
                        print("3")
                else:
                    pass
        cv.imshow('Circulos detectados', frame)
        if cv.waitKey(1) == ord('q'):
            return False
        return True
    
    def run(self):
        while self.detect_traffic_light():
            pass
        self.cap.release()
        cv.destroyAllWindows()

if __name__ == "__main__":
    detector = TrafficLightDetector()
    detector.run()
