import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray, Point
from std_msgs.msg import Empty
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transitions import Machine
import open3d as o3d

from parasight.sam_ui import SegmentAnythingUI
from parasight.registration import RegistrationPipeline
from parasight.utils import *

import time

from parasight_interfaces.srv import StartTracking
from parasight_interfaces.msg import TrackedPoints
from std_srvs.srv import Empty as EmptySrv
from functools import partial

from ament_index_python.packages import get_package_share_directory

class ParaSightHost(Node):
    states = ['waiting', 'ready', 'user_input', 'tracker_active', 'system_paused', 'stabilizing', 'lock_in']

    def __init__(self):
        super().__init__('parasight_host')
        
        # Create state machine
        self.machine = Machine(model=self, states=ParaSightHost.states, initial='waiting',
                               after_state_change='publish_state')

        # Transitions
        self.machine.add_transition(trigger='all_systems_ready', source='waiting', dest='ready')
        self.machine.add_transition(trigger='start_parasight', source='ready', dest='user_input')
        self.machine.add_transition(trigger='input_received', source='user_input', dest='tracker_active')
        self.machine.add_transition(trigger='tracking_lost', source='tracker_active', dest='system_paused')
        self.machine.add_transition(trigger='tracking_restored', source='system_paused', dest='stabilizing')
        self.machine.add_transition(trigger='stabilized', source='stabilizing', dest='tracker_active')
        self.machine.add_transition(trigger='ready_to_drill', source='tracker_active', dest='lock_in')
        self.machine.add_transition(trigger='drill_complete', source='lock_in', dest='ready')

        self.machine.add_transition(trigger='hard_reset', source='*', dest='waiting')

        # State Data
        self.last_rgb_image = None
        self.last_depth_image = None
        self.last_cloud = None
        self.annotated_points = None
        self.need_to_register = True

        # Set up tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Trigger Subscribers
        self.ui_trigger_subscription = self.create_subscription(
            Empty,
            '/trigger_host_ui',
            self.ui_trigger_callback,
            10)
        self.hard_reset_subscription = self.create_subscription(
            Empty,
            '/hard_reset_host',
            self.hard_reset_callback,
            10)
        
        # Data Subscribers
        self.rgb_image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.rgb_image_callback,
            10)
        self.depth_image_subscription = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_image_callback,
            10)
        self.pcd_subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.pcd_callback,
            10)
        self.tracked_points_subscription = self.create_subscription(
            TrackedPoints,
            '/tracked_points',
            self.tracked_points_callback,
            10)
        
        # Publishers
        self.pcd_publisher = self.create_publisher(PointCloud2, '/processed_point_cloud', 10)
        self.pose_array_publisher = self.create_publisher(PoseArray, '/localized_pose_array', 10)

        # Service Clients
        self.start_tracking_client = self.create_client(StartTracking, '/start_tracking')
        self.stop_tracking_client = self.create_client(EmptySrv, '/stop_tracking')

        # Load resources
        package_dir = get_package_share_directory('parasight')
        self.source = o3d.io.read_point_cloud(package_dir + "/resource/femur_shell.ply")
        self.plan_path = package_dir + "/resource/plan_config.yaml"
        self.plan = "plan3"

        # Interfaces
        self.regpipe = RegistrationPipeline()
        self.segmentation_ui = SegmentAnythingUI()
        self.bridge = CvBridge()

        # Create timer to check systems
        self.create_timer(1.0, self.check_all_systems)

    def check_all_systems(self):
        if self.state == 'waiting':
            if self.all_systems_ready():
                self.trigger('all_systems_ready')

    def all_systems_ready(self, dummy=True):
        if dummy:
            return True
            
        try:
            # Check tf transform
            self.tf_buffer.lookup_transform('world', 'camera_color_optical_frame', rclpy.time.Time())
            
            # Check required nodes
            node_names = [node.split('/')[-1] for node in self.get_node_names()]
            if 'io_node' not in node_names or 'registration_node' not in node_names:
                return False
                
            return True
            
        except TransformException:
            return False

    def publish_state(self):
        self.get_logger().info(f'Current state: {self.state}')

    def ui_trigger_callback(self, msg):
        self.get_logger().info('UI trigger received')
        if self.state == 'ready':
            self.trigger('start_parasight')
            mask, annotated_points, mask_points = self.segmentation_ui.segment_using_ui(self.last_rgb_image) # Blocking call
            self.annotated_points = annotated_points
            self.trigger('input_received')
        else:
            self.get_logger().warn('UI trigger received but not in ready state')

    def on_enter_tracker_active(self):
        assert self.annotated_points is not None
        # Convert points to geometry_msgs/Point
        ros_points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in self.annotated_points]
        
        # Prepare the request
        request = StartTracking.Request(points=ros_points)
        
        # Call asynchronously to avoid blocking
        self.start_tracking_client.wait_for_service()
        self.future = self.start_tracking_client.call_async(request)
        self.future.add_done_callback(partial(self.tracking_response_callback))

    def tracking_response_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Failed to start tracking")
            else:
                self.get_logger().info("Tracking started")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def hard_reset_callback(self, msg):
        self.get_logger().info('Hard reset received, attempting to stop tracking...')
        
        # Create an empty request for the stop_tracking service
        stop_tracking_request = EmptySrv.Request()
        
        # Call stop_tracking service asynchronously
        self.stop_tracking_client.wait_for_service()
        future = self.stop_tracking_client.call_async(stop_tracking_request)
        
        # Add a callback to handle after stopping tracking
        future.add_done_callback(partial(self.after_stop_tracking))

    def after_stop_tracking(self, future):
        try:
            # Check if the service call was successful
            future.result()  # This will raise an exception if the service failed
            self.get_logger().info("Tracking successfully stopped, proceeding with reset.")
        except Exception as e:
            self.get_logger().error(f"Failed to stop tracking: {e}")
        
        # Now, trigger the state machine reset
        self.trigger('hard_reset')

    def rgb_image_callback(self, msg):
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        self.last_rgb_image = rgb_image

    def depth_image_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1") / 1000.0
        self.last_depth_image = depth_image

    def add_depth(self,points):
        new_points = []
        for point in points:
            x,y = point
            depth = self.last_depth_image[int(y),int(x)]
            new_points.append([x,y,depth])
        return new_points
    
    def pcd_callback(self, msg):
        cloud = from_msg(msg)
        self.last_cloud = cloud

    def tracked_points_callback(self, msg):
        if not self.state == 'tracker_active':
            return
        
        if not msg.stable or not msg.all_visible:
            self.need_to_register = True
            return # Wait for tracking to stabilize
        elif self.need_to_register:
            self.get_logger().info('Registering...')
            annotated_points = [(p.x, p.y) for p in msg.points]
            t0 = time.time()
            mask, annotated_points, mask_points = self.segmentation_ui.segment_using_points(self.last_rgb_image, annotated_points[0], annotated_points[1])
            annotated_points = self.add_depth(annotated_points)
            mask_points = self.add_depth(mask_points)
            source_cloud = self.source.voxel_down_sample(voxel_size=0.003)
            transform = self.regpipe.register(mask, source_cloud, self.last_cloud, annotated_points, mask_points)
            t1 = time.time()
            self.get_logger().info(f'Registration time: {t1 - t0}')

            source_cloud.transform(transform)
            source_cloud.paint_uniform_color([1, 0, 0])
            self.publish_point_cloud(source_cloud)
            self.need_to_register = False

    def publish_point_cloud(self, cloud):
        cloud_msg = to_msg(cloud,frame_id='camera_depth_optical_frame')
        self.pcd_publisher.publish(cloud_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ParaSightHost()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
