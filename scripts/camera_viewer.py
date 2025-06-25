#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector_node', anonymous=True)
        self.bridge = CvBridge()

        # Dicionário com os limites de cor em HSV
        # ATENÇÃO: Estes valores podem precisar de ajuste fino!
        self.colors = {
            'vermelho': ([0, 120, 70], [10, 255, 255]),
            'verde': ([36, 100, 100], [86, 255, 255]),
            'azul': ([100, 150, 50], [130, 255, 255])       
 }

        # Assinando o tópico da câmera
        self.image_sub = rospy.Subscriber("/ufba_camera/image_raw", Image, self.callback)
        rospy.loginfo("Nó detector de cores iniciado. Aguardando imagens...")

    def callback(self, data):
        try:
            # Converte a imagem ROS para o formato OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(f"Erro na conversão do cv_bridge: {e}")
            return

        # Converte a imagem de BGR para HSV (melhor para detecção de cor)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Itera sobre cada cor que queremos detectar
        for color_name, (lower, upper) in self.colors.items():
            # Cria a máscara para a cor atual
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

            # Encontra os contornos na máscara
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # Desenha um retângulo e o nome da cor para cada contorno encontrado
            for cnt in contours:
                # Filtra pequenos ruídos ignorando contornos de área pequena
                area = cv2.contourArea(cnt)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(cnt)
                    # Desenha o retângulo na imagem original
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # Escreve o nome da cor
                    cv2.putText(cv_image, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # Mostra a imagem final com as detecções
        cv2.imshow("Color Detection", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        ColorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
