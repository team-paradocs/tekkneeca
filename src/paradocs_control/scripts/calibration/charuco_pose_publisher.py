import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2.aruco as aruco

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class CharucoPosePublisher(Node):
    def __init__(self):
        super().__init__('charuco_pose_publisher')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.image_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.publisher = self.create_publisher(PoseStamped, '/marker_1_frame', 10)
        self.bridge = CvBridge()
        # self.aruco_dict = aruco.Dictionary(aruco.DICT_6X6_100, _markerSize=0.015)
        # self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_dict = aruco.getPredefinedDictionary(0)
        self.charuco_board = aruco.CharucoBoard((6,6), 0.03, 0.024, self.aruco_dict)
        self.camera_matrix = None  # Load your camera matrix
        self.dist_coeffs = None  # Load your distortion coefficients

    def camera_info_callback(self, camera_info_msg):
        # Extract camera intrinsic matrix and distortion coefficients from camera info
        self.camera_matrix = np.array(camera_info_msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(camera_info_msg.d)
        # self.get_logger().info(f"Received camera info")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)
        
        # print(ids)
        
        if ids is not None:
            # print('Markers detected')
            detector_params = aruco.CharucoParameters()
            detector_params.cameraMatrix = self.camera_matrix 
            detector_params.distCoeffs = self.dist_coeffs
            # _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(corners, ids, gray, self.charuco_board)
            charuco_detector = aruco.CharucoDetector(self.charuco_board, detector_params)
            charuco_corners = charuco_ids = None
            charuco_corners, charuco_ids, charuco_marker_corners, charuco_marker_ids = charuco_detector.detectBoard(gray, charuco_corners, 
                                                                                                                    charuco_ids, corners, ids)
            # print(charuco_corners, charuco_ids)
            if charuco_corners is not None and charuco_ids is not None:
                # print('Charuco detected')
                frame = aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)

            if charuco_ids is not None and len(charuco_ids) > 3:
                # print('Charuco detected')
                rvec = np.zeros((3,1))
                tvec = np.zeros((3,1))
                retval = aruco.estimatePoseCharucoBoard(charuco_corners, charuco_ids, self.charuco_board, self.camera_matrix, self.dist_coeffs,rvec, tvec)
                frame = aruco.drawDetectedCornersCharuco(frame, charuco_corners, charuco_ids)
                if retval:
                    # frame = aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    pose = PoseStamped()
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.header.frame_id = 'camera_color_optical_frame'
                    # pose.header.frame_id = 'marker_1_frame'
                    # pose.header.reference_frame = 'camera_color_optical_frame'
                    pose.pose.position.x = tvec[0][0]
                    pose.pose.position.y = tvec[1][0]
                    pose.pose.position.z = tvec[2][0]
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    r = R.from_matrix(rotation_matrix)
                    quaternion = r.as_quat()
                    pose.pose.orientation.x = quaternion[0]
                    pose.pose.orientation.y = quaternion[1]
                    pose.pose.orientation.z = quaternion[2]
                    pose.pose.orientation.w = quaternion[3]
                    self.publisher.publish(pose)
                    # print('Published pose', pose)

                    t = TransformStamped()

                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'camera_color_optical_frame'
                    t.child_frame_id = 'marker_1_frame'
                    t.transform.translation.x = tvec[0][0]
                    t.transform.translation.y = tvec[1][0]
                    t.transform.translation.z = tvec[2][0]
                    t.transform.rotation.x = quaternion[0]
                    t.transform.rotation.y = quaternion[1]
                    t.transform.rotation.z = quaternion[2]
                    t.transform.rotation.w = quaternion[3]

                    self.tf_broadcaster.sendTransform(t)

        cv2.imshow('frame', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CharucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()