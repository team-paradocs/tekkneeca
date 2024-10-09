import cv2.detail
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_matrix
import math


class PointSelectionNode(Node):
    def __init__(self):
        super().__init__('point_selection_node')
        
        # Subscribing to camera feed and camera info topics
        self.image_sub = self.create_subscription(Image, '/camera/color/image_rect_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.depth_image_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_image_callback, 10)

        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.selected_points = []
        self.camera_frame = None
        # self.points = [
        #     [-0.5, 0.0, 0.05],
        #     [-0.52, 0.0, 0.05],
        #     [-0.48, 0.0, 0.05],
        #     [-0.5, 0.02, 0.05],
        #     [-0.5, -0.02, 0.05],
        #     [-0.52, -0.02, 0.05],
        #     [-0.52, 0.02, 0.05],
        #     [-0.48, -0.02, 0.05],
        #     [-0.48, 0.02, 0.05],
        # ]
        self.points = [
            [-0.50, 0.00, 0.051],
            [-0.45, 0.00, 0.051],
            [-0.47, 0.02, 0.051],
            [-0.47, -0.02, 0.051]
        ]
        self.points_index = 0
        self.image_resize_scale = 2
        self.save_frame = 1


        self.depth_image = None
        self.rgb_image = None
        self.camera_frame_points = None
        self.extrinsic_matrix = None
        self.ee_transform = None
        self.selected_pixel = None

        


        self.orientation_id = 0

    def depth_image_callback(self, depth_image_msg):
        # Convert the ROS Image message to a CV2 image
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')
        # print(self.depth_image.shape)
        # self.get_logger().info("Received depth image")

    def camera_info_callback(self, camera_info_msg):
        # Extract camera intrinsic matrix and distortion coefficients from camera info
        self.camera_matrix = np.array(camera_info_msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(camera_info_msg.d)
        # self.get_logger().info(f"Received camera info")



    def image_callback(self, image_msg):
        if self.camera_matrix is None:
            self.get_logger().warn("Camera matrix is not available yet.")
            return
        
        if self.depth_image is None:
            return
        
        
        # Convert the ROS Image message to a CV2 image
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        self.rgb_image = cv_image

        self.camera_frame = image_msg.header.frame_id
        # Write the instruction text on the image
        instruction_text = f"Select point corresponding to the coordinates: {self.points[self.points_index]}"
        cv2.putText(cv_image, instruction_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Resize the image
        cv_image = cv2.resize(cv_image, (cv_image.shape[1] * self.image_resize_scale, cv_image.shape[0] * self.image_resize_scale))
        # Display image and allow point selection
        cv2.imshow("Camera Frame", cv_image)

        cv2.setMouseCallback("Camera Frame", self.on_mouse_click, cv_image)
        cv2.waitKey(10)

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            x = math.floor(x/ self.image_resize_scale)
            y = math.floor (y/ self.image_resize_scale)
            
            #destroy cv2 window
            self.selected_points.append((x, y))
            self.selected_pixel = [x,y]
            # self.get_logger().info(f"Selected point: {x}, {y}")

            # Convert pixel coordinates to 3D point in camera frame
            # cv2.destroyAllWindows()
            z = self.depth_image[y][x] / 1000
            # print(z)
            point_in_camera_frame = self.pixel_to_camera_frame(x, y, z)
            if point_in_camera_frame is not None:
                # self.get_logger().info(f"Point in camera frame: {point_in_camera_frame}")
                
                # Transform point to base frame
                self.transform_to_base_frame(point_in_camera_frame)

    def pixel_to_camera_frame(self, x, y, depth):
        
        if self.camera_matrix is None:
            self.get_logger().warn("Camera matrix not available.")
            return None

        # Unproject pixel to camera coordinates using intrinsic matrix
        inv_camera_matrix = np.linalg.inv(self.camera_matrix)
        pixel_coords = np.array([x, y, 1.0])
        camera_coords = depth * np.dot(inv_camera_matrix, pixel_coords)

        self.camera_frame_points = camera_coords

        return camera_coords

    def transform_to_base_frame(self, point_in_camera_frame):
        try:
            # Transform point from camera frame to base frame using tf2
            now = rclpy.time.Time()
            #print all available frames
            
            # frames = self.tf_buffer.all_frames_as_string()
            # print(frames)

            try:

                transform = self.tf_buffer.lookup_transform('lbr/link_0', self.camera_frame, rclpy.time.Time())

                self.extrinsic_matrix = self.tf_buffer.lookup_transform('lbr/link_ee', self.camera_frame, rclpy.time.Time())
                self.ee_transform = self.tf_buffer.lookup_transform('lbr/link_0', 'lbr/link_ee', rclpy.time.Time())
            except:
                print("transform error")
                return
            
            # Convert transform to matrix
            trans = transform.transform.translation
            rot = transform.transform.rotation
            transformation_matrix = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
            transformation_matrix[0:3, 3] = [trans.x, trans.y, trans.z]

            # Apply transformation to the point
            point_in_camera_frame_homogeneous = np.array([point_in_camera_frame[0], point_in_camera_frame[1], point_in_camera_frame[2], 1])
            point_in_base_frame = np.dot(transformation_matrix, point_in_camera_frame_homogeneous)

            point_in_base_frame = point_in_base_frame[:3]
            # self.get_logger().info(f"Point in base frame: {point_in_base_frame[:3]}")
            # print(point_in_base_frame)

            # Calculate error between the transformed point and the original point
            original_point = self.points[self.points_index]
            error = np.linalg.norm(point_in_base_frame[:2] - original_point[:2])

            mm = error *1000

            if self.save_frame == 1:
                file1 = open('calibration_data.csv', 'a')  # append mode
                file1.write(f"Orientation {self.orientation_id}\n")
                file1.close()

                extrinsic = transform_to_pq(self.extrinsic_matrix.transform)
                transform_ee =  transform_to_pq(self.ee_transform.transform)
                file1 = open('calibration_transforms.csv', 'a')  # append mode
                file1.write(f"{transform_ee};{extrinsic};{self.camera_matrix.tolist()} \n")
                file1.close()

                # Save the depth and RGB images using the orientation_id
                depth_image_filename = f'caliibration_images/depth_image_{self.orientation_id}.png'
                rgb_image_filename = f'caliibration_images/rgb_image_{self.orientation_id}.png'
                
                cv2.imwrite(depth_image_filename, self.depth_image)
                cv2.imwrite(rgb_image_filename, self.rgb_image)

                self.save_frame = 0



            file1 = open('calibration_data.csv', 'a')  # append mode
            file1.write(f"{original_point};{point_in_base_frame[:3].tolist()};{self.camera_frame_points.tolist()};{self.selected_pixel};{error};{mm}\n")
            file1.close()
            
            print(f"{original_point},{point_in_base_frame[:3].tolist()},{self.camera_frame_points.tolist()}, {error}, {mm}")

            # Move to the next point
            self.points_index += 1
            if self.points_index >= len(self.points):
                self.get_logger().info("All points processed.")
                # cv2.destroyAllWindows()
                self.points_index = 0
                self.save_frame = 1
                self.orientation_id = self.orientation_id + 1
            
            

        except Exception as e:
            self.get_logger().error(f"Failed to transform point: {e}")

def transform_to_pq(msg):
    """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = [msg.translation.x, msg.translation.y, msg.translation.z]
    q = [msg.rotation.x, msg.rotation.y,
                  msg.rotation.z, msg.rotation.w]
    return [p, q]


def main(args=None):
    rclpy.init(args=args)
    node = PointSelectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
