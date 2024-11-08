import cv2
import torch
import rclpy
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # Import Point message type
import time

class RGBTracker(Node):
    def __init__(self):
        super().__init__('rgb_tracker')

        # CoTracker Setup
        self.device = torch.device("cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu")
        model = torch.hub.load("facebookresearch/co-tracker", "cotracker3_online")
        self.model = model.to(self.device)
        self.buffer = []
        self.buffer_size = 16
        self.is_first_step = True
        self.tracking_enabled = False
        self.point_selected = False
        self.queries = None

        # ROS Setup
        self.declare_parameter("sub_topic", "/camera/color/image_rect_raw")
        self.declare_parameter("pub_topic", "/camera/color/image_track")

        self.sub_topic = self.get_parameter("sub_topic").get_parameter_value().string_value
        self.pub_topic = self.get_parameter("pub_topic").get_parameter_value().string_value

        # Subscribe to image and annotated point topics
        self.image_subscription = self.create_subscription(
            Image,
            self.sub_topic,
            self.image_callback,
            10
        )
        self.point_subscription = self.create_subscription(
            Point,
            '/annotated_point',  # Topic to receive point coordinates for tracking
            self.point_callback,
            10
        )

        self.publisher_ = self.create_publisher(Image, self.pub_topic, 10)
        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.logger.info("RGB Tracker initialized")

    def point_callback(self, point_msg):
        # Set tracking point from received message and initialize queries
        self.queries = torch.tensor([[0, point_msg.x, point_msg.y]], dtype=torch.float32).to(self.device)
        self.point_selected = True
        self.tracking_enabled = True  # Enable tracking once a point is received
        self.logger.info(f"Tracking point received: ({point_msg.x}, {point_msg.y})")

    def process_buffer(self):
        # Process video buffer using the CoTracker model
        video_chunk = torch.tensor(np.stack(self.buffer), device=self.device).float().permute(0, 3, 1, 2)[None]
        pred_tracks, pred_visibility = self.model(video_chunk, self.is_first_step, queries=self.queries[None])
        self.is_first_step = False
        return pred_tracks, pred_visibility

    def image_callback(self, msg):
        # Process images only if tracking is enabled and point is selected
        if not self.tracking_enabled or not self.point_selected:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.buffer.append(cv_image)

        self.logger.info(f"Buffer size: {len(self.buffer)}")
        if len(self.buffer) == self.buffer_size:
            # Perform tracking
            pred_tracks, _ = self.process_buffer()
            self.buffer = self.buffer[self.buffer_size // 2:]

            # Visualize the tracking result on the image
            if pred_tracks is not None:
                updated_point = pred_tracks[0][-1][0]
                x, y = int(updated_point[0]), int(updated_point[1])
                self.logger.info(f"Tracking point: ({x}, {y})")

                # Annotate image with tracking point
                cv2.circle(cv_image, (x, y), 10, (0, 0, 255), -1)
                cv2.putText(cv_image, "Tracking", (x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Publish annotated image
                annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
                self.publisher_.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    rgb_tracker = RGBTracker()
    
    # Spin the node to keep it running
    rclpy.spin(rgb_tracker)

    # Clean up
    rgb_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
