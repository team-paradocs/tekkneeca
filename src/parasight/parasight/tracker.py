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
from parasight_interfaces.srv import StartTracking
from parasight_interfaces.msg import TrackedPoints
from std_srvs.srv import Empty

class Tracker(Node):
    def __init__(self):
        super().__init__('tracker')

        # CoTracker Setup
        self.device = torch.device("cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu")
        model = torch.hub.load("facebookresearch/co-tracker", "cotracker3_online")
        self.model = model.to(self.device)
        self.buffer = []
        self.buffer_size = 16
        self.stability_thresh = 3
        self.is_first_step = True
        self.tracking_enabled = False
        self.queries = None

        # ROS Setup
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.image_callback,
            10
        )

        self.pose_publisher = self.create_publisher(PoseStamped, '/tracked_pose', 10)
        self.publisher_ = self.create_publisher(Image, '/camera/color/image_track', 10)
        self.tracked_points_publisher = self.create_publisher(TrackedPoints, '/tracked_points', 10)

        self.start_service = self.create_service(StartTracking, '/start_tracking', self.start_tracking_callback)
        self.stop_service = self.create_service(Empty, '/stop_tracking', self.stop_tracking_callback)
        
        self.bridge = CvBridge()
        self.logger = self.get_logger()

        self.logger.info("RGB Tracker initialized")

    def start_tracking_callback(self, request, response):
        self.tracking_enabled = True
        points = request.points
        self.queries = torch.tensor([[0, point.x, point.y] for point in points], dtype=torch.float32).to(self.device)
        self.logger.info(f"Starting tracking with {len(points)} points")
        response.success = True
        return response

    def stop_tracking_callback(self, request, response):
        self.tracking_enabled = False
        self.queries = None
        self.buffer = []
        self.is_first_step = True
        self.logger.info("Stopping tracking")
        return response


    def process_buffer(self):
        # Process video buffer using the CoTracker model
        video_chunk = torch.tensor(np.stack(self.buffer), device=self.device).float().permute(0, 3, 1, 2)[None]
        pred_tracks, pred_visibility = self.model(video_chunk, self.is_first_step, queries=self.queries[None])
        self.is_first_step = False
        return pred_tracks, pred_visibility

    def image_callback(self, msg):
        # Process images only if tracking is enabled
        if not self.tracking_enabled:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.buffer.append(cv_image)

        if len(self.buffer) == self.buffer_size:
            # Perform tracking
            # t0 = time.time()
            pred_tracks, visibility = self.process_buffer()
            # tracking_time = (time.time() - t0) * 1000  # Convert to milliseconds
            # fps = 1000 / tracking_time  # Calculate FPS
            # self.logger.info(f"Average inference time: {tracking_time:.2f}ms, FPS: {fps:.2f}")
            self.buffer = self.buffer[self.buffer_size // 2:]

            # Visualize the tracking result on the image
            if pred_tracks is not None:
                # Initialize previous positions if not already done
                if not hasattr(self, 'prev_positions'):
                    self.prev_positions = [(int(point[0]), int(point[1])) for point in pred_tracks[0][-1]]
                    self.stable = False

                # Process each tracked point
                current_positions = []
                all_stable = True
                
                for i, point in enumerate(pred_tracks[0][-1]):
                    x, y = int(point[0]), int(point[1])
                    current_positions.append((x, y))
                    
                    # Calculate stability for this point
                    prev_x, prev_y = self.prev_positions[i]
                    delta = np.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
                    point_stable = delta < self.stability_thresh
                    all_stable = all_stable and point_stable

                    # Annotate each point
                    thickness = -1 if visibility[0][-1][i] else 2
                    color = (0, 255, 0) if point_stable else (0, 0, 255)
                    cv2.circle(cv_image, (x, y), 10, color, thickness)
                    status = "Stable" if point_stable else "Moving"
                    cv2.putText(cv_image, f"Point {i} ({status}) | ({delta:.2f})", 
                              (x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # self.logger.info(f"Tracking point {i}: ({x}, {y})")

                # Update stability and previous positions
                self.stable = all_stable
                self.prev_positions = current_positions

                # Publish annotated image
                annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
                self.publisher_.publish(annotated_msg)

                # Publish TrackedPoints
                tracked_points_msg = TrackedPoints()
                tracked_points_msg.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in current_positions]
                tracked_points_msg.visibility = visibility[0][-1].tolist()
                tracked_points_msg.stable = bool(self.stable)
                tracked_points_msg.all_visible = all(tracked_points_msg.visibility)
                self.tracked_points_publisher.publish(tracked_points_msg)



def main(args=None):
    rclpy.init(args=args)
    tracker = Tracker()
    
    # Spin the node to keep it running
    rclpy.spin(tracker)

    # Clean up
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
