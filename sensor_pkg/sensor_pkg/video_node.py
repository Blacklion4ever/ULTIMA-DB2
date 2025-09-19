import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess

# Import du service généré dans le package CMake
from sensor_msgs_pkg.srv import SetVideoControls

DEVICE = "/dev/video2"

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # --- Publisher vidéo ---
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(1/30, self.timer_callback)  # 30 FPS
        self.bridge = CvBridge()

        # --- Initialisation caméra ---
        subprocess.run(["v4l2-ctl", "--device="+DEVICE, "--set-standard=NTSC"])
        self.set_controls(brightness=150, contrast=70, saturation=80, hue=0)

        self.cap = cv2.VideoCapture(DEVICE)
        if not self.cap.isOpened():
            self.get_logger().error(f"Impossible d'ouvrir la caméra {DEVICE}")

        # --- Service pour régler la caméra ---
        self.srv = self.create_service(
            SetVideoControls,
            'set_video_controls',
            self.set_video_controls_callback
        )
        self.get_logger().info("Service 'set_video_controls' prêt à recevoir des commandes.")

    # --- Fonction pour régler les paramètres de la caméra ---
    def set_controls(self, brightness=None, contrast=None, saturation=None, hue=None):
        if brightness is not None:
            subprocess.run(["v4l2-ctl", "--device="+DEVICE, f"--set-ctrl=brightness={brightness}"])
        if contrast is not None:
            subprocess.run(["v4l2-ctl", "--device="+DEVICE, f"--set-ctrl=contrast={contrast}"])
        if saturation is not None:
            subprocess.run(["v4l2-ctl", "--device="+DEVICE, f"--set-ctrl=saturation={saturation}"])
        if hue is not None:
            subprocess.run(["v4l2-ctl", "--device="+DEVICE, f"--set-ctrl=hue={hue}"])

    # --- Callback du service ---
    def set_video_controls_callback(self, request, response):
        self.get_logger().info(
            f"Réglages reçus -> Brightness: {request.brightness}, "
            f"Contrast: {request.contrast}, "
            f"Saturation: {request.saturation}, "
            f"Hue: {request.hue}"
        )

        try:
            self.set_controls(
                brightness=request.brightness,
                contrast=request.contrast,
                saturation=request.saturation,
                hue=request.hue
            )
            response.success = True
            response.message = "Réglages appliqués avec succès."
            self.get_logger().info("Réglages appliqués avec succès.")
        except Exception as e:
            response.success = False
            response.message = f"Erreur lors de l'application des réglages : {str(e)}"
            self.get_logger().error(response.message)

        return response

    # --- Capture et publication de la vidéo ---
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame non capturée")
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

    # --- Nettoyage ---
    def destroy(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

