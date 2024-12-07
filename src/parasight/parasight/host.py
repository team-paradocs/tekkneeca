import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray, Point, Pose, PoseStamped, PointStamped
from std_msgs.msg import Empty
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose, do_transform_point
from std_msgs.msg import Int32
from transitions import Machine
import open3d as o3d

from parasight.segment_ui import SegmentAnythingUI
from parasight.registration import RegistrationPipeline
from parasight.utils import *

import time

from parasight_interfaces.srv import StartTracking, StopTracking
from parasight_interfaces.msg import TrackedPoints
from std_srvs.srv import Empty as EmptySrv
from functools import partial

from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R

from visualization_msgs.msg import Marker


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
        self.spam_counter = 0
        self.last_drill_pose_array = None
        self.last_drill_pose = None
        
        # D405 Intrinsics
        self.fx = 425.19189453125
        self.fy = 424.6562805175781
        self.cx = 422.978515625
        self.cy = 242.1155242919922


        # Set up tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'lbr/link_0'
        self.ee_frame = 'lbr/link_ee'
        self.tool_frame = 'lbr/link_tool'
        self.camera_link_frame = 'camera_link'
        self.camera_frame = 'camera_depth_optical_frame'


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
        self.reg_request_subscription = self.create_subscription(
            Int32,
            '/trigger_reg',
            self.reg_request_callback,
            10)
        self.log_request_subscription = self.create_subscription(
            Empty,
            '/log_request',
            self.log_request_callback,
            10)
        
        # Publishers
        self.pcd_publisher = self.create_publisher(PointCloud2, '/processed_point_cloud', 10)
        self.pose_array_publisher = self.create_publisher(PoseArray, '/surgical_drill_pose', 10)
        self.marker_publisher = self.create_publisher(Marker, '/fitness_marker', 10)

        # Service Clients
        self.start_tracking_client = self.create_client(StartTracking, '/start_tracking')
        self.stop_tracking_client = self.create_client(StopTracking, '/stop_tracking')

        # Parameters
        self.declare_parameter('selected_bones', 'both')
        self.update_bones(self.get_parameter('selected_bones').value)
        self.add_on_set_parameters_callback(self.parameter_change_callback)

        # Load resources
        package_dir = get_package_share_directory('parasight')
        femur_cloud = o3d.io.read_point_cloud(package_dir + "/resource/femur_shell.ply")
        tibia_cloud = o3d.io.read_point_cloud(package_dir + "/resource/tibia_shell.ply")
        self.sources = {'femur': femur_cloud, 'tibia': tibia_cloud}
        self.colors = {'femur': [1, 0, 0], 'tibia': [0, 0, 1]}
        self.plan_path = package_dir + "/resource/plan_config_v2.yaml"
        self.plan = "plan1"

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
            masks, annotated_points, all_mask_points = self.segmentation_ui.segment_using_ui(self.last_rgb_image, self.bones) # Blocking call
            self.annotated_points = annotated_points
            self.trigger('input_received')
        else:
            self.get_logger().warn('UI trigger received but not in ready state')

    def on_enter_tracker_active(self):
        assert self.annotated_points is not None
        # Convert points to geometry_msgs/Point
        ros_points = [Point(x=float(p[0]), y=float(p[1]), z=0.0) for p in self.annotated_points]
        
        # Prepare the request
        request = StartTracking.Request(points=ros_points, resume=False)
        
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
        stop_tracking_request = StopTracking.Request(reset=True)
        
        # Call stop_tracking service asynchronously
        self.stop_tracking_client.wait_for_service()
        future = self.stop_tracking_client.call_async(stop_tracking_request)
        
        # Add a callback to handle after stopping tracking
        future.add_done_callback(partial(self.after_stop_tracking, reset=True))

    def after_stop_tracking(self, future, reset=True):
        try:
            # Check if the service call was successful
            future.result()  # This will raise an exception if the service failed
            self.get_logger().info("Tracking successfully stopped, proceeding with reset." if reset else "Tracking successfully paused.")
        except Exception as e:
            self.get_logger().error(f"Failed to stop tracking: {e}")
        
        # Now, trigger the state machine reset
        if reset:
            self.trigger('hard_reset')

    def pause_tracking(self):
        request = StopTracking.Request(reset=False)
        self.stop_tracking_client.wait_for_service()
        self.future = self.stop_tracking_client.call_async(request)
        self.future.add_done_callback(partial(self.after_stop_tracking, reset=False))

    def resume_tracking(self):
        request = StartTracking.Request(resume=True)
        self.start_tracking_client.wait_for_service()
        self.future = self.start_tracking_client.call_async(request)
        self.future.add_done_callback(partial(self.tracking_response_callback))

    def update_bones(self, selected_bones):
        """Update self.bones based on the /selected_bones parameter."""
        if selected_bones == 'femur':
            self.bones = ['femur']
        elif selected_bones == 'tibia':
            self.bones = ['tibia']
        elif selected_bones == 'both':
            self.bones = ['femur', 'tibia']
        else:
            self.get_logger().warn(f"Unknown /selected_bones value: {selected_bones}, defaulting to empty list")
            self.bones = []
        self.get_logger().info(f"Updated bones to: {self.bones}")
    
    def parameter_change_callback(self, params):
        for param in params:
            if param.name == 'selected_bones':
                self.update_bones(param.value)
        return SetParametersResult(successful=True)

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
            depth = average_depth(self.last_depth_image,y,x)
            new_points.append([x,y,depth])
        return new_points
    
    def pcd_callback(self, msg):
        cloud = from_msg(msg)
        self.last_cloud = cloud

    def pose_direction(self, annotated_points):
        # Dynamically compute theta
        # If Femur is on the left, theta should be 0
        # If Femur is on the right, theta should be -pi
        femur_point_x = annotated_points[0][0]
        tibia_point_x = annotated_points[1][0]
        if femur_point_x > tibia_point_x:
            return 0
        else:
            return -np.pi


    def register_and_publish(self, points):
        # self.pause_tracking() # Pause tracking while registering to improve performance
        masks, annotated_points, all_mask_points = self.segmentation_ui.segment_using_points(self.last_rgb_image, points[0], points[1], self.bones)
        self.annotated_points = annotated_points # cache
        annotated_points = self.add_depth(annotated_points)
        registered_clouds = []
        transforms = {}
        for i, bone in enumerate(self.bones):
            t0 = time.time()
            mask = masks[i]
            mask_points = all_mask_points[i]
            source = self.sources[bone]
            mask_points = self.add_depth(mask_points)
            transform, fitness = self.regpipe.register(mask, source, self.last_cloud, annotated_points, mask_points, bone=bone)
            t1 = time.time()
            self.get_logger().info(f'Registration time for {bone}: {t1 - t0}')
            source_cloud = source.voxel_down_sample(voxel_size=0.003)
            source_cloud.transform(transform)
            source_cloud.paint_uniform_color(self.colors[bone])
            registered_clouds.append(source_cloud)
            transforms[bone] = transform

        self.publish_point_cloud(registered_clouds)

        theta = self.pose_direction(annotated_points)
        drill_pose_array = self.compute_plan(transform,theta=theta)
        drill_pose_array = self.calibration_offset(drill_pose_array,0.004,-0.001,0.0)
        drill_pose_array.header.frame_id = self.base_frame
        drill_pose_array.header.stamp = self.get_clock().now().to_msg()
        self.pose_array_publisher.publish(drill_pose_array)
        self.last_drill_pose_array = drill_pose_array
        self.spam_counter = 3

        # DEPRECATED
        self.last_drill_pose = drill_pose_array.poses[0]
        # self.publish_fitness_marker(self.last_drill_pose.position, fitness)
        # self.resume_tracking()

    def publish_fitness_marker(self, position, fitness):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fitness_marker"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z + 0.1  # Offset slightly above the first pose
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.05  # Text size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f"{fitness:.2f}"  # Format the fitness value
        self.marker_publisher.publish(marker)

    def reg_request_callback(self, msg):
        if msg.data == 0:
            if self.annotated_points is not None:
                print('Re-Registering...')
                self.register_and_publish(self.annotated_points)
            else:
                self.get_logger().warn('No annotated points set. Register with UI first')
        elif msg.data == 1:
            print('Re-Registering with UI...')
            masks, annotated_points, all_mask_points = self.segmentation_ui.segment_using_ui(self.last_rgb_image, self.bones) # Blocking call
            self.annotated_points = annotated_points
            self.register_and_publish(annotated_points)

    def tracked_points_callback(self, msg):
        if not self.state == 'tracker_active':
            return
        
        if not msg.all_visible:
            self.get_logger().info('Not all points are visible')
            return
        
        if self.spam_counter > 0 and self.last_drill_pose_array is not None:
            self.spam_counter -= 1
            self.pose_array_publisher.publish(self.last_drill_pose_array)
            return
        
        if not msg.stable:
            self.need_to_register = True
            return # Wait for tracking to stabilize
        elif self.need_to_register:
            annotated_points = [(p.x, p.y) for p in msg.points]
            self.get_logger().info('Registering...')
            self.register_and_publish(annotated_points)
            self.need_to_register = False

    def publish_point_cloud(self, clouds):
        # Combine all clouds into one
        combined_cloud = o3d.geometry.PointCloud()
        for c in clouds:
            combined_cloud += c
        cloud_msg = to_msg(combined_cloud,frame_id=self.camera_frame)
        self.pcd_publisher.publish(cloud_msg)

    def compute_plan(self, transforms,theta=-np.pi/2):
        """Computes the surgical drill point by transforming the default point with the given transform."""

        drill_pose_array = PoseArray()

        parts = load_plan_points(self.plan_path, self.plan)
        for bone, holes in parts.items():
            if bone not in self.bones:
                continue

            transform = transforms[bone]
            for hole_name, hole in holes.items():
                p1, p2, p3 = hole

                # Toggle theta between 0 and -π/2 for femur curvature hole
                curr_theta = theta
                if bone == 'femur' and hole_name == 'hole3':
                    curr_theta = np.pi if theta == 0 else 0
                elif bone == 'tibia':
                    curr_theta = theta * -1

                mesh = o3d.geometry.TriangleMesh()
                mesh.vertices = o3d.utility.Vector3dVector([p1, p2, p3])
                mesh.triangles = o3d.utility.Vector3iVector([[0, 1, 2]])
                mesh.compute_vertex_normals()
                normal =  np.asarray(mesh.vertex_normals)[0]
                actual_normal = -normal
                z_axis = np.array([0, 0, 1])
                rotation_axis = np.cross(z_axis, actual_normal)
                rotation_axis /= np.linalg.norm(rotation_axis)
                angle = np.arccos(np.dot(z_axis, actual_normal) / (np.linalg.norm(z_axis) * np.linalg.norm(actual_normal)))
                Rot = o3d.geometry.get_rotation_matrix_from_axis_angle(rotation_axis * angle)

                T = np.eye(4)
                T[:3, :3] = Rot
                T[:3, 3] = p3

                default_plan = T
                default_plan_rotated = np.copy(default_plan)
                default_plan_rotated[:3, :3] = np.matmul(default_plan_rotated[:3, :3], R.from_euler('z', curr_theta).as_matrix())

                T = np.matmul(transform, default_plan_rotated)

                # Convert R to a quaternion
                Rot = T[:3, :3]
                r = R.from_matrix(Rot)
                quat = r.as_quat()  # Returns (x, y, z, w)

                p = T[:3, 3]

                pose = Pose()
                pose.position.x = p[0]
                pose.position.y = p[1]
                pose.position.z = p[2]

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                base_transform = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rclpy.time.Time())
                pose = do_transform_pose(pose, base_transform)

                drill_pose_array.poses.append(pose)

        return drill_pose_array

    def calibration_offset(self,pose_array,x_off,y_off,z_off):
        for pose in pose_array.poses:
            pose.position.x += x_off
            pose.position.y += y_off
            pose.position.z += z_off
        return pose_array

    def log_request_callback(self, msg):
        self.get_logger().info('Log requested')
        base_to_tool = self.tf_buffer.lookup_transform(self.base_frame, self.tool_frame, rclpy.time.Time())
        self.get_logger().info(f'Base to tool: {base_to_tool}')
        self.get_logger().info(f'Drill pose: {self.last_drill_pose}') # Target in World Frame


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
