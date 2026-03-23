#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class VisualCortex(Node):
    def __init__(self):
        super().__init__('visual_cortex')
        self.bridge = CvBridge()
        
        # 1. Setup Publisher for RViz2
        self.det_pub = self.create_publisher(CompressedImage, '/brain/debug_feed/compressed', 10)
        
        # 2. Connect to DroidCam (CHANGE IP HERE)
        self.droidcam_url = "http://192.168.x.x:4747/video"
        self.cap = cv2.VideoCapture(self.droidcam_url)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Could not connect to DroidCam at {self.droidcam_url}")
            return
            
        self.get_logger().info('✅ DroidCam Connected!')

        # 3. Load YOLO onto the GPU
        self.get_logger().info('Loading YOLO26m...')
        self.model = YOLO('yolo26m.pt').to('cuda')
        
        # 4. Create a timer to constantly read frames (approx 30 FPS)
        self.timer = self.create_timer(0.033, self.process_frame)
        self.get_logger().info('ARES-X: Brain Online & Broadcasting to RViz2...')

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Dropped frame from DroidCam")
            return
        
        # Inference
        results = self.model.predict(frame, conf=0.4, device='cuda', half=True, verbose=False)
        annotated_frame = results[0].plot()
        
        # Convert to ROS message and publish to RViz2
        det_msg = self.bridge.cv2_to_compressed_imgmsg(annotated_frame)
        det_msg.header.stamp = self.get_clock().now().to_msg()
        det_msg.header.frame_id = "camera" 
        self.det_pub.publish(det_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualCortex()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()