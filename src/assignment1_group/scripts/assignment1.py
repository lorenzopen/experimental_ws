#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection
from cv_bridge import CvBridge
import cv2
import numpy as np

class AssignmentController(Node):
    def __init__(self):
        super().__init__('assignment_controller')

        # --- PARAMETRI & TOPIC ---
        # Aggiornati in base al tuo 'ros2 topic list'
        self.cmd_vel_topic = '/cmd_vel'
        self.aruco_topic = '/aruco_detections'
        self.camera_topic = '/camera/image'        # <--- CORRETTO (era image_raw)
        self.cam_info_topic = '/camera/camera_info'

        # --- SUBSCRIBERS ---
        # 1. Detection (Dati sui marker)
        self.sub_detection = self.create_subscription(
            ArucoDetection,
            self.aruco_topic,
            self.detection_callback,
            10)

        # 2. Immagine (Solo per disegno cerchio)
        self.sub_image = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)

        # 3. Camera Info (Per la matrice K)
        self.sub_info = self.create_subscription(
            CameraInfo,
            self.cam_info_topic,
            self.info_callback,
            10)

        # --- PUBLISHERS ---
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_processed_img = self.create_publisher(Image, '/output/image_marker', 10)

        # --- TIMER LOOP ---
        # Fondamentale: esegue la logica ogni 0.1 secondi (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # --- VARIABILI INTERNE ---
        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_matrix = None
        
        self.current_detections = None # Buffer per l'ultimo msg ricevuto
        
        self.found_ids = set()      # Insieme degli ID unici trovati
        self.target_ids = []        # Lista ordinata degli ID da visitare
        self.current_target_idx = 0
        self.state = 'SCANNING'     # Stati: SCANNING -> ALIGNING -> DONE

        self.get_logger().info("Nodo avviato correttamente. Inizio fase SCANNING...")

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.get_logger().info("Matrice Camera calibrata ricevuta.")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Errore conversione immagine: {e}")

    def detection_callback(self, msg):
        # Salviamo solo il dato, non blocchiamo la callback con logica pesante
        self.current_detections = msg

    def control_loop(self):
        """Loop principale di controllo (chiamato dal Timer)"""
        twist = Twist()

        # --- STATO 1: SCANNING (Cerca i marker ruotando) ---
        if self.state == 'SCANNING':
            # Ruota sempre, anche se non vede nulla
            twist.angular.z = 0.3 

            # Analizza detection se disponibili
            if self.current_detections:
                for marker in self.current_detections.markers:
                    if marker.marker_id not in self.found_ids:
                        self.found_ids.add(marker.marker_id)
                        self.get_logger().info(f"SCANNING: Trovato nuovo ID {marker.marker_id}. Totale: {len(self.found_ids)}/5")

            # Condizione di uscita: trovati 5 marker
            if len(self.found_ids) >= 5: #
                self.get_logger().info("Tutti i marker trovati! Calcolo percorso...")
                self.target_ids = sorted(list(self.found_ids)) # Ordine crescente
                self.get_logger().info(f"Sequenza di visita: {self.target_ids}")
                
                self.state = 'ALIGNING'
                twist.angular.z = 0.0 # Stop
                self.current_detections = None # Pulisci buffer

        # --- STATO 2: ALIGNING (Visual Servoing) ---
        elif self.state == 'ALIGNING':
            # Se abbiamo finito la lista
            if self.current_target_idx >= len(self.target_ids):
                self.state = 'DONE'
                return

            target_id = self.target_ids[self.current_target_idx]
            target_marker = None

            # Cerca il target specifico nel frame corrente
            if self.current_detections:
                for marker in self.current_detections.markers:
                    if marker.marker_id == target_id:
                        target_marker = marker
                        break
            
            if target_marker:
                # Logica di controllo P (Proporzionale)
                x_pos = target_marker.pose.position.x
                
                # Formula: vel_z = -Kp * errore_x
                twist.angular.z = -0.8 * x_pos 
                
                # Se è abbastanza centrato (tolleranza 5cm)
                if abs(x_pos) < 0.05:
                    twist.angular.z = 0.0
                    self.get_logger().info(f"Target ID {target_id} CENTRATO!")
                    
                    # Disegna e pubblica immagine
                    self.publish_marker_image(target_marker)
                    
                    # Passa al prossimo
                    self.current_target_idx += 1
                    self.current_detections = None # Reset per evitare doppi conteggi rapidi
            else:
                # Se sono in ALIGNING ma ho perso il marker, ruoto piano per ritrovarlo
                twist.angular.z = 0.3

        # --- STATO 3: DONE ---
        elif self.state == 'DONE':
            twist.angular.z = 0.0
            # self.get_logger().info("Missione completata.")

        # Invia comando ai motori
        self.pub_cmd_vel.publish(twist)

    def publish_marker_image(self, marker):
        """Disegna il cerchio attorno al marker e pubblica l'immagine"""
        if self.latest_image is None or self.camera_matrix is None:
            return

        try:
            # Proiezione 3D -> 2D
            p_cam = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
            
            # Formule: u = (fx * x / z) + cx, v = (fy * y / z) + cy
            u = int((p_cam[0] * self.camera_matrix[0,0] / p_cam[2]) + self.camera_matrix[0,2])
            v = int((p_cam[1] * self.camera_matrix[1,1] / p_cam[2]) + self.camera_matrix[1,2])
            
            # Crea copia per non sporcare l'originale
            processed_img = self.latest_image.copy()
            
            # Disegna cerchio verde e testo
            cv2.circle(processed_img, (u, v), 30, (0, 255, 0), 4)
            cv2.putText(processed_img, f"ID {marker.marker_id}", (u+40, v), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            out_msg = self.bridge.cv2_to_imgmsg(processed_img, encoding="bgr8")
            self.pub_processed_img.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f"Errore disegno overlay: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AssignmentController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()