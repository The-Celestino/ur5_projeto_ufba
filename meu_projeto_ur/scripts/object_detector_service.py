#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point, Quaternion

# Importe a definição do seu serviço
# Substitua 'seu_pacote' pelo nome real do seu pacote
from seu_pacote.srv import DetectObjects, DetectObjectsResponse

class ObjectDetectorService:
    def __init__(self):
        rospy.init_node('object_detector_service_node')
        self.bridge = CvBridge()
        
        # Buffer e Listener do TF para transformações de coordenadas
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Limites de cor em HSV
        self.colors = {
            'red_cube': ([0, 120, 70], [10, 255, 255]),
            'green_cube': ([36, 100, 100], [86, 255, 255]),
            'blue_cube': ([100, 150, 50], [130, 255, 255])
        }

        # Subscriber para a informação da câmera (só precisamos uma vez)
        self.camera_info = rospy.wait_for_message("/ufba_camera/camera_info", CameraInfo)
        rospy.loginfo("Informações da câmera recebidas.")
        
        # Inicia o Serviço ROS
        self.service = rospy.Service('detect_objects', DetectObjects, self.handle_detection_request)
        
        rospy.loginfo("Serviço de detecção de objetos pronto.")
        rospy.spin()

    def handle_detection_request(self, req):
        rospy.loginfo("Requisição de detecção recebida.")
        
        # Pega a imagem de cor e de profundidade mais recentes
        try:
            rgb_image_msg = rospy.wait_for_message("/ufba_camera/image_raw", Image, timeout=2.0)
            depth_image_msg = rospy.wait_for_message("/ufba_camera/depth/image_raw", Image, timeout=2.0)
            
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8")
            # Converta a imagem de profundidade para um formato que o numpy possa ler
            cv_depth = self.bridge.imgmsg_to_cv2(depth_image_msg, "32FC1") # ou "16UC1" dependendo do seu sensor

        except Exception as e:
            rospy.logerr(f"Erro ao receber imagens: {e}")
            return None

        hsv_image = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2HSV)

        detected_names = []
        detected_poses = []

        for color_name, (lower, upper) in self.colors.items():
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Pega o maior contorno para evitar ruído
                cnt = max(contours, key=cv2.contourArea)
                if cv2.contourArea(cnt) > 500:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        # Centro do objeto em pixels
                        cx_pixel = int(M["m10"] / M["m00"])
                        cy_pixel = int(M["m01"] / M["m00"])
                        
                        # Pega a profundidade (distância) em metros
                        depth_m = cv_depth[cy_pixel, cx_pixel]

                        # Se a profundidade for inválida (ex: 0 ou NaN), pula
                        if np.isnan(depth_m) or depth_m == 0:
                            continue
                        
                        # Converte pixel (u,v) + profundidade (d) para coordenadas 3D no frame da câmera
                        K = self.camera_info.K
                        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
                        x_cam = (cx_pixel - cx) * depth_m / fx
                        y_cam = (cy_pixel - cy) * depth_m / fy
                        z_cam = depth_m

                        # Transforma o ponto 3D do frame da câmera para o frame do robô
                        try:
                            # Frame de destino para o qual queremos as coordenadas
                            target_frame = "base_link"
                            # Frame de origem (o frame da sua câmera)
                            source_frame = rgb_image_msg.header.frame_id
                            
                            pose_in_camera = Pose()
                            pose_in_camera.position = Point(x_cam, y_cam, z_cam)
                            pose_in_camera.orientation = Quaternion(0,0,0,1) # Orientação neutra

                            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
                            pose_in_world = tf2_geometry_msgs.do_transform_pose(pose_in_camera, transform)

                            detected_names.append(color_name)
                            detected_poses.append(pose_in_world)
                            rospy.loginfo(f"Objeto {color_name} encontrado na posição (mundo): {pose_in_world.position.x:.3f}, {pose_in_world.position.y:.3f}, {pose_in_world.position.z:.3f}")

                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                            rospy.logerr(f"Erro de transformação TF: {e}")

        return DetectObjectsResponse(detected_names, detected_poses)

if __name__ == '__main__':
    ObjectDetectorService()