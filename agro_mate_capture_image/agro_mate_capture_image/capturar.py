import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
import imutils

class Ros2OpenCVImageConverter(Node):   

    def __init__(self):
        super().__init__('Ros2OpenCVImageConverter')
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/image',
            self.camera_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
    def camera_callback(self, data):
        try:
            # Seleccionamos bgr8 porque es la codificacion de OpenCV por defecto
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #---AMARILLO---#
        amarillo_osc = np.array([25, 78, 120])
        amarillo_cla = np.array([30, 255, 255])

        #---ROJO---#
        rojo_osc = np.array([0, 50, 120])
        rojo_cla = np.array([10, 255, 255])

        #---VERDE---#
        verde_osc = np.array([40, 70, 80])
        verde_cla = np.array([70, 255, 255])

        #---AZUL---#
        azul_osc = np.array([98, 60, 0])
        azul_cla = np.array([121, 255, 255])

        #---NARANJA---#
        naranja_osc = np.array([10, 100, 100])
        naranja_cla = np.array([25, 255, 255])

        # Crear mÃ¡scaras para cada color
        cara1 = cv2.inRange(hsv, amarillo_osc, amarillo_cla)
        cara2 = cv2.inRange(hsv, rojo_osc, rojo_cla)
        cara3 = cv2.inRange(hsv, verde_osc, verde_cla)
        cara4 = cv2.inRange(hsv, azul_osc, azul_cla)
        cara5 = cv2.inRange(hsv, naranja_osc, naranja_cla)

        cnts1 = cv2.findContours(cara1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts1 = imutils.grab_contours(cnts1)

        cnts2 = cv2.findContours(cara2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts2 = imutils.grab_contours(cnts2)

        cnts3 = cv2.findContours(cara3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts3 = imutils.grab_contours(cnts3)

        cnts4 = cv2.findContours(cara4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts4 = imutils.grab_contours(cnts4)

        cnts5 = cv2.findContours(cara5, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts5 = imutils.grab_contours(cnts5)

        # Dibuja contornos y etiqueta objetos en la imagen
        for c in cnts1:
            area1 = cv2.contourArea(c)
            if area1 > 5000:
                cv2.drawContours(cv_image, [c], -1, (30, 255, 255), 3)
                M = cv2.moments(c)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(cv_image, (cx, cy), 7, (255, 255, 255), -1)
                cv2.putText(cv_image, "No es una naranja", (cx - 20, cy - 20), cv2.FONT_ITALIC, 2, (255, 255, 255), 2)

        for c in cnts2:
            area2 = cv2.contourArea(c)
            if area2 > 5000:
                cv2.drawContours(cv_image, [c], -1, (0, 0, 255), 3)
                M = cv2.moments(c)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(cv_image, (cx, cy), 7, (255, 255, 255), -1)
                cv2.putText(cv_image, "No es una naranja", (cx - 20, cy - 20), cv2.FONT_ITALIC, 2, (255, 255, 255), 2)

        for c in cnts3:
            area3 = cv2.contourArea(c)
            if area3 > 5000:
                cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 3)
                M = cv2.moments(c)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(cv_image, (cx, cy), 7, (255, 255, 255), -1)
                cv2.putText(cv_image, "No es una naranja", (cx - 20, cy - 20), cv2.FONT_ITALIC, 2, (255, 255, 255), 2)

        for c in cnts4:
            area4 = cv2.contourArea(c)
            if area4 > 5000:
                cv2.drawContours(cv_image, [c], -1, (255, 0, 0), 3)
                M = cv2.moments(c)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(cv_image, (cx, cy), 7, (255, 255, 255), -1)
                cv2.putText(cv_image, "No es una naranja", (cx - 20, cy - 20), cv2.FONT_ITALIC, 2, (255, 255, 255), 2)
        
        for c in cnts5:
            area5 = cv2.contourArea(c)
            if area5 > 5000:
                cv2.drawContours(cv_image, [c], -1, (0, 165, 255), 3)  # Color naranja en BGR
                M = cv2.moments(c)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(cv_image, (cx, cy), 7, (255, 255, 255), -1)
                cv2.putText(cv_image, "Es una naranja", (cx - 20, cy - 20), cv2.FONT_ITALIC, 0.5, (255, 255, 255), 2)

        cv2.imshow("Imagen capturada por el robot", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    img_converter_object = Ros2OpenCVImageConverter()
    try:
        rclpy.spin(img_converter_object)
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()