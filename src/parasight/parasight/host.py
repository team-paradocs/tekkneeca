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

from parasight.sam_ui import SegmentAnythingUI
from parasight.utils import *

from parasight_interfaces.srv import StartTracking
from std_srvs.srv import Empty as EmptySrv

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
        
        # Publishers
        self.pcd_publisher = self.create_publisher(PointCloud2, '/processed_point_cloud', 10)
        self.pose_array_publisher = self.create_publisher(PoseArray, '/localized_pose_array', 10)

        # Service Clients
        self.start_tracking_client = self.create_client(StartTracking, '/start_tracking')
        self.stop_tracking_client = self.create_client(EmptySrv, '/stop_tracking')

        # Interfaces
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
        self.get_logger().info(f"Current state: {self.state}")
        # Convert points to geometry_msgs/Point
        ros_points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in self.annotated_points]
        response = self.start_tracking_client.call(StartTracking.Request(points=ros_points))
        if not response.success:
            self.get_logger().error("Failed to start tracking")
        else:
            self.get_logger().info("Tracking started")

    def hard_reset_callback(self, msg):
        self.get_logger().info('Hard reset received')
        self.trigger('hard_reset')

    def rgb_image_callback(self, msg):
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        self.last_rgb_image = rgb_image

    def depth_image_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1") / 1000.0
        self.last_depth_image = depth_image

    def pcd_callback(self, msg):
        cloud = from_msg(msg)
        self.last_cloud = cloud


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
