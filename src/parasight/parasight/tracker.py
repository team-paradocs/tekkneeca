import cv2
import torch
import rclpy
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped, PoseStamped
import time
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from parasight_interfaces.srv import StartTracking, StopTracking
from parasight_interfaces.msg import TrackedPoints
from std_srvs.srv import Empty

from parasight.utils import *

from ament_index_python.packages import get_package_share_directory
from cotracker.predictor import CoTrackerOnlinePredictor

class Tracker(Node):
    def __init__(self):
        super().__init__('tracker')

        # D405 Intrinsics
        self.fx = 425.19189453125
        self.fy = 424.6562805175781
        self.cx = 422.978515625
        self.cy = 242.1155242919922
        self.camera_frame = 'camera_color_optical_frame'
        self.base_frame = 'lbr/link_0'
        self.point_labels = ['FEMUR', 'TIBIA']
        self.base_color = (17,52,68) # RGB


        # CoTracker Setup
        self.device = torch.device("cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu")
        # model = torch.hub.load("facebookresearch/co-tracker", "cotracker3_online") # To get the latest model. Currently breaks
        model_path = get_package_share_directory('parasight') + "/checkpoints/scaled_online.pth"
        model = CoTrackerOnlinePredictor(checkpoint=model_path)
        self.model = model.to(self.device)
        self.buffer = []
        self.buffer_size = 16
        self.stability_thresh = 3
        self.is_first_step = True
        self.tracking_enabled = False
        self.queries = None
        self.last_depth_image = None

        self.stable_and_published = False

        # ROS Setup
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.image_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_image_callback,
            10
        )

        self.pose_publisher = self.create_publisher(PoseStamped, '/tracked_pose', 10)
        self.publisher_ = self.create_publisher(Image, '/camera/color/image_track', 10)
        self.tracked_points_publisher = self.create_publisher(TrackedPoints, '/tracked_points', 10)

        self.start_service = self.create_service(StartTracking, '/start_tracking', self.start_tracking_callback)
        self.stop_service = self.create_service(StopTracking, '/stop_tracking', self.stop_tracking_callback)
        
        self.bridge = CvBridge()
        self.logger = self.get_logger()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.logger.info("RGB Tracker initialized")

    def start_tracking_callback(self, request, response):
        self.tracking_enabled = True
        if not request.resume:  
            points = request.points
            self.queries = torch.tensor([[0, point.x, point.y] for point in points], dtype=torch.float32).to(self.device)
            self.logger.info(f"Starting tracking with {len(points)} points")
        else:
            self.logger.info("Resuming tracking")
        response.success = True
        return response

    def stop_tracking_callback(self, request, response):
        self.tracking_enabled = False
        if request.reset:
            self.queries = None
            self.buffer = []
            self.is_first_step = True
            self.logger.info("Stopping tracking")
        else:
            self.logger.info("Pausing tracking")
        response.success = True
        return response


    def process_buffer(self):
        # Process video buffer using the CoTracker model
        video_chunk = torch.tensor(np.stack(self.buffer), device=self.device).float().permute(0, 3, 1, 2)[None]
        pred_tracks, pred_visibility = self.model(video_chunk, self.is_first_step, queries=self.queries[None])
        self.is_first_step = False
        return pred_tracks, pred_visibility
    
    def depth_image_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1") / 1000.0
        self.last_depth_image = depth_image

    def pixel_to_3d(self, x, y):
        z = average_depth(self.last_depth_image,y,x)
        x = (x - self.cx) * z / self.fx
        y = (y - self.cy) * z / self.fy

        point_3d = PointStamped()
        point_3d.header.frame_id = self.camera_frame
        point_3d.point.x = x
        point_3d.point.y = y
        point_3d.point.z = z

        transform = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rclpy.time.Time())
        point_3d = do_transform_point(point_3d, transform)

        pose = PoseStamped()
        hover_offset = 0.1
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = point_3d.point.x
        pose.pose.position.y = point_3d.point.y
        pose.pose.position.z = point_3d.point.z + hover_offset

        # Fixed Orientation
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        return pose

    def image_callback(self, msg):
        # Process images only if tracking is enabled
        if not self.tracking_enabled:
            # Passthrough
            self.publisher_.publish(msg)
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
                    thickness = -1
                    if not visibility[0][-1][i]:
                        color = (0, 0, 255)  # Red if not visible
                    elif not point_stable:
                        color = (0, 255, 255)  # Yellow if not stable
                    else:
                        color = (0, 255, 0)  # Green if visible and stable
                    # Draw filled circle
                    cv2.circle(cv_image, (x, y), 10, color, thickness, cv2.LINE_AA)
                    # Draw black outline
                    cv2.circle(cv_image, (x, y), 10, self.base_color, 1, cv2.LINE_AA)
                    cv2.putText(cv_image, self.point_labels[i], (x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.base_color, 2, cv2.LINE_AA)
                    self.draw_legend(cv_image)
                    # status = "Stable" if point_stable else "Moving"
                    # cv2.putText(cv_image, f"Point {i} ({status}) | ({delta:.2f})", 
                    #           (x + 15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
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


                # Publish Tracked Pose if not stable
                # If stable, publish only once
                # if not all visible, don't publish
                if not tracked_points_msg.all_visible:
                    return
                
                x,y = tracked_points_msg.points[0].x, tracked_points_msg.points[0].y
                if self.stable:
                    if not self.stable_and_published:
                        self.pose_publisher.publish(self.pixel_to_3d(x, y))
                        self.stable_and_published = True
                else:
                    self.pose_publisher.publish(self.pixel_to_3d(x, y))
                    self.stable_and_published = False

    def draw_legend(self, image):
        cv2.circle(image, (30, 20), 10, (0,255,0), -1, cv2.LINE_AA)
        cv2.circle(image, (30, 20), 10, self.base_color, 1, cv2.LINE_AA)
        cv2.putText(image, "Stable", (45, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.base_color, 2, cv2.LINE_AA)
        cv2.circle(image, (30, 50), 10, (0,255,255), -1, cv2.LINE_AA)
        cv2.circle(image, (30, 50), 10, self.base_color, 1, cv2.LINE_AA)
        cv2.putText(image, "Moving", (45, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.base_color, 2, cv2.LINE_AA)
        cv2.circle(image, (30, 80), 10, (0,0,255), -1, cv2.LINE_AA)
        cv2.circle(image, (30, 80), 10, self.base_color, 1, cv2.LINE_AA)
        cv2.putText(image, "Not Visible", (45, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.base_color, 2, cv2.LINE_AA)



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
