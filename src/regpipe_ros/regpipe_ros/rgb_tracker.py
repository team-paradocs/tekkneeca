import cv2
import torch
import rclpy
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped, PoseStamped
import time
import tf2_geometry_msgs
import tf2_ros

class RGBTracker(Node):
    def __init__(self):
        super().__init__('rgb_tracker')

        # D405 Intrinsics for 3D Projection
        self.fx = 425.19189453125
        self.fy = 424.6562805175781
        self.cx = 422.978515625
        self.cy = 242.1155242919922

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
        self.last_depth_image = None

        # ROS Setup
        self.declare_parameter("sub_topic", "/camera/color/image_rect_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("pub_topic", "/camera/color/image_track")
        self.declare_parameter("pose_topic", "/tracked_pose")

        self.sub_topic = self.get_parameter("sub_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.pub_topic = self.get_parameter("pub_topic").get_parameter_value().string_value
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value


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
        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.pose_publisher = self.create_publisher(PoseStamped, self.pose_topic, 10)

        self.publisher_ = self.create_publisher(Image, self.pub_topic, 10)
        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.logger.info("RGB Tracker initialized")


    def pixel_to_3d(self, x, y):
        z = self.last_depth_image[y, x]
        x = (x - self.cx) * z / self.fx
        y = (y - self.cy) * z / self.fy

        point_3d = PointStamped()
        point_3d.header.frame_id = "camera_color_optical_frame"
        point_3d.point.x = x
        point_3d.point.y = y
        point_3d.point.z = z

        transform = self.tf_buffer.lookup_transform("world", "camera_color_optical_frame", rclpy.time.Time())
        point_3d = tf2_geometry_msgs.do_transform_point(point_3d, transform)

        pose = PoseStamped()
        hover_offset = 0.15
        pose.header.frame_id = "world"
        pose.pose.position.x = point_3d.point.x
        pose.pose.position.y = point_3d.point.y
        pose.pose.position.z = point_3d.point.z + hover_offset

        # Fixed Orientation
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        return pose

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
        if not self.tracking_enabled or not self.point_selected or self.last_depth_image is None:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.buffer.append(cv_image)

        # self.logger.info(f"Buffer size: {len(self.buffer)}")
        if len(self.buffer) == self.buffer_size:
            # Perform tracking
            t0 = time.time()
            pred_tracks, _ = self.process_buffer()
            tracking_time = (time.time() - t0) * 1000  # Convert to milliseconds
            fps = 1000 / tracking_time  # Calculate FPS
            # self.logger.info(f"Average inference time: {tracking_time:.2f}ms, FPS: {fps:.2f}")
            self.buffer = self.buffer[self.buffer_size // 2:]

            # Visualize the tracking result on the image
            if pred_tracks is not None:
                updated_point = pred_tracks[0][-1][0]
                x, y = int(updated_point[0]), int(updated_point[1])
                # self.logger.info(f"Tracking point: ({x}, {y})")

                # Annotate image with tracking point
                cv2.circle(cv_image, (x, y), 10, (0, 0, 255), -1)
                cv2.putText(cv_image, "Tracked Point", (x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # Publish annotated image
                annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
                self.publisher_.publish(annotated_msg)

                # Publish pose
                pose = self.pixel_to_3d(x, y)
                self.pose_publisher.publish(pose)

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1") / 1000.0
        self.last_depth_image = depth_image

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
