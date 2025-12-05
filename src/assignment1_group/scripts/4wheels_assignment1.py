#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class AssignmentController(Node):
    def __init__(self):
        super().__init__('assignment_controller')

        # =================================================================
        # CONFIGURAZIONE TOPIC E PARAMETRI
        # =================================================================
        self.topic_camera_img = '/camera/image' 
        self.topic_camera_info = '/camera/camera_info'
        
        # Topic del nodo di visione (ros_aruco_opencv)
        self.topic_aruco = '/aruco_detections'
        
        # Topic di movimento
        self.topic_cmd_vel = '/cmd_vel'

        # Velocità e Guadagni
        self.scan_speed = 0.4        # Velocità di rotazione durante la ricerca
        self.k_p = 1.0               # Guadagno proporzionale per il centraggio (aumenta se il robot è pigro)
        self.center_tolerance = 0.05 # Tolleranza allineamento (metri rispetto al centro ottico)
        # =================================================================

        # --- SUBSCRIBERS ---
        # 1. Detection Marker (Best Effort per performance)
        self.sub_detection = self.create_subscription(
            ArucoDetection,
            self.topic_aruco,
            self.detection_callback,
            qos_profile_sensor_data)

        # 2. Immagine Camera (Per disegnare il cerchio)
        self.sub_image = self.create_subscription(
            Image,
            self.topic_camera_img,
            self.image_callback,
            10)

        # 3. Info Camera (Per la matrice di proiezione 3D -> 2D)
        self.sub_info = self.create_subscription(
            CameraInfo,
            self.topic_camera_info,
            self.info_callback,
            10)

        # --- PUBLISHERS ---
        self.pub_cmd_vel = self.create_publisher(Twist, self.topic_cmd_vel, 10)
        # Topic richiesto dall'assignment per l'immagine modificata 
        self.pub_processed_img = self.create_publisher(Image, '/output/image_marker', 10)

        # --- TIMER (Loop di Controllo) ---
        # Esegue la logica ogni 0.1 secondi (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # --- VARIABILI INTERNE ---
        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_matrix = None       # Matrice intrinseca K
        self.current_detections = None  # Ultimo messaggio ArUco ricevuto
        
        # Gestione Stati
        self.found_ids = set()      # Set per ID unici trovati
        self.target_ids = []        # Lista ordinata da visitare
        self.current_target_idx = 0
        self.state = 'SCANNING'     # Stati: SCANNING -> ALIGNING -> DONE

        self.get_logger().info("Nodo Controller Avviato. Stato iniziale: SCANNING")

    def info_callback(self, msg):
        """Salva la matrice di calibrazione della telecamera"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.get_logger().info("Matrice Camera (K) ricevuta.")

    def image_callback(self, msg):
        """Salva l'ultima immagine raw per poterci disegnare sopra"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Errore conversione immagine: {e}")

    def detection_callback(self, msg):
        """Riceve le posizioni dei marker dal pacchetto aruco_opencv"""
        self.current_detections = msg

    def control_loop(self):
        """Logica principale a stati finiti (Visual Servoing) [cite: 3]"""
        twist = Twist()

        # -----------------------------------------------------------------
        # FASE 1: SCANNING (Esplorazione)
        # -----------------------------------------------------------------
        if self.state == 'SCANNING':
            # Ruota per cercare i marker 
            twist.angular.z = self.scan_speed 

            # Se vedo marker, li aggiungo alla lista dei trovati
            if self.current_detections:
                for marker in self.current_detections.markers:
                    if marker.marker_id not in self.found_ids:
                        self.found_ids.add(marker.marker_id)
                        self.get_logger().info(f"Trovato ID {marker.marker_id}. Totale: {len(self.found_ids)}/5")

            # Condizione uscita: Trovati 5 marker (come richiesto dall'assignment) [cite: 4]
            if len(self.found_ids) >= 5:
                self.get_logger().info("Tutti i marker trovati! Ordinamento lista...")
                # Ordina gli ID in ordine crescente 
                self.target_ids = sorted(list(self.found_ids))
                self.get_logger().info(f"Sequenza target: {self.target_ids}")
                
                self.state = 'ALIGNING'
                twist.angular.z = 0.0
                self.current_detections = None # Reset buffer

        # -----------------------------------------------------------------
        # FASE 2: ALIGNING (Visual Servoing)
        # -----------------------------------------------------------------
        elif self.state == 'ALIGNING':
            # Controllo se abbiamo finito la lista
            if self.current_target_idx >= len(self.target_ids):
                self.state = 'DONE'
                return

            target_id = self.target_ids[self.current_target_idx]
            target_marker = None

            # Cerco il target corrente nel messaggio di detection
            if self.current_detections:
                for marker in self.current_detections.markers:
                    if marker.marker_id == target_id:
                        target_marker = marker
                        break
            
            if target_marker:
                # --- LOGICA DI CONTROLLO ---
                # x_pos è la coordinata orizzontale nel frame della camera.
                # x=0 è il centro. x>0 è destra, x<0 è sinistra.
                x_pos = target_marker.pose.position.x
                
                # Controllo Proporzionale: ruota contro l'errore
                # Se il marker è a destra (x positivo), ruoto a destra (z negativo)
                twist.angular.z = -self.k_p * x_pos 

                # Debug opzionale
                # self.get_logger().info(f"Target {target_id}: errore={x_pos:.3f}, comando={twist.angular.z:.3f}")
                
                # Controllo se è centrato (entro la tolleranza) [cite: 3]
                if abs(x_pos) < self.center_tolerance:
                    twist.angular.z = 0.0
                    self.get_logger().info(f"Target ID {target_id} CENTRATO!")
                    
                    # 1. Disegna cerchio e pubblica immagine custom 
                    self.publish_marker_circle(target_marker)
                    
                    # 2. Pausa scenica e passaggio al prossimo
                    time.sleep(1.0) # Piccola pausa per stabilizzare
                    self.current_target_idx += 1
                    self.current_detections = None 
            else:
                # Se sono in fase di allineamento ma non vedo il target,
                # ruoto piano per cercarlo nel campo visivo
                twist.angular.z = self.scan_speed

        # -----------------------------------------------------------------
        # FASE 3: FINE
        # -----------------------------------------------------------------
        elif self.state == 'DONE':
            twist.angular.z = 0.0
            # Stop del robot

        # Pubblica il comando di velocità calcolato
        self.pub_cmd_vel.publish(twist)

    def publish_marker_circle(self, marker):
        """Disegna un cerchio attorno al marker e pubblica sul topic custom"""
        if self.latest_image is None or self.camera_matrix is None:
            return

        try:
            # Proiezione Punto 3D -> Pixel 2D
            # Coordinate del marker nel frame camera
            p_cam = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
            
            # Formule Pinhole Camera:
            # u = (fx * x) / z + cx
            # v = (fy * y) / z + cy
            fx = self.camera_matrix[0,0]
            cx = self.camera_matrix[0,2]
            fy = self.camera_matrix[1,1]
            cy = self.camera_matrix[1,2]

            u = int((p_cam[0] * fx / p_cam[2]) + cx)
            v = int((p_cam[1] * fy / p_cam[2]) + cy)
            
            # Copia immagine per non sovrascrivere il buffer originale
            processed_img = self.latest_image.copy()
            
            # Disegna cerchio (Verde, raggio 30, spessore 4)
            cv2.circle(processed_img, (u, v), 30, (0, 255, 0), 4)
            
            # Aggiungi testo ID
            cv2.putText(processed_img, f"ID {marker.marker_id}", (u+40, v), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Converti e pubblica
            out_msg = self.bridge.cv2_to_imgmsg(processed_img, encoding="bgr8")
            self.pub_processed_img.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f"Errore nel disegno overlay: {e}")

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