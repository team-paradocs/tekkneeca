# Description: This script is used to detect the charuco board and publish the pose of the charuco board in the camera frame.
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


class CharucoPosePublisher(Node):
    def __init__(self):
        super().__init__("charuco_pose_publisher")
        self.subscription = self.create_subscription(
            Image, "/camera/color/image_rect_raw", self.image_callback, 10
        )

        self.D435_subscription = self.create_subscription(
            Image, "/D435/color/image_raw", self.D435image_callback, 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.camera_info_callback, 10
        )
        self.D435_camera_info_sub = self.create_subscription(
            CameraInfo, "/D435/color/camera_info", self.D435_camera_info_callback, 10
        )

        self.publisher = self.create_publisher(PoseStamped, "/marker_1_frame", 10)
        self.D435publisher = self.create_publisher(PoseStamped, "/marker_frame_2", 10)

        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(0)
        self.charuco_board = aruco.CharucoBoard((6, 6), 0.03, 0.024, self.aruco_dict)

        self.camera_matrix = None  # Load your camera matrix
        self.dist_coeffs = None  # Load your distortion coefficients
        self.D435_camera_matrix = None  # Load your camera matrix
        self.D435_dist_coeffs = None  # Load your distortion coefficients

    def camera_info_callback(self, camera_info_msg):
        # Extract camera intrinsic matrix and distortion coefficients from camera info
        self.camera_matrix = np.array(camera_info_msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(camera_info_msg.d)

    def D435_camera_info_callback(self, camera_info_msg):
        # Extract camera intrinsic matrix and distortion coefficients from camera info
        self.D435_camera_matrix = np.array(camera_info_msg.k).reshape(3, 3)
        self.D435_dist_coeffs = np.array(camera_info_msg.d)

    def D435image_callback(self, msg):
        frame_id = "D435_color_optical_frame"
        marker_frame = "marker_frame_2"
        self.detect_charuco(msg, frame_id, "D435", marker_frame)

    def image_callback(self, msg):
        frame_id = "camera_color_optical_frame"
        marker_frame = "marker_1_frame"
        self.detect_charuco(msg, frame_id, "D405", marker_frame)

    def detect_charuco(self, msg, frame_id, camera_name, marker_frame):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)
        if ids is not None:
            frame = aruco.drawDetectedMarkers(frame, corners, ids)

        if ids is not None:
            detector_params = aruco.CharucoParameters()
            if camera_name == "D405":

                detector_params.cameraMatrix = self.camera_matrix
                detector_params.distCoeffs = self.dist_coeffs
                camera_matrix = self.camera_matrix
                dist_coeffs = self.dist_coeffs
            else:
                detector_params.cameraMatrix = self.D435_camera_matrix
                detector_params.distCoeffs = self.D435_dist_coeffs
                camera_matrix = self.D435_camera_matrix
                dist_coeffs = self.D435_dist_coeffs
            charuco_detector = aruco.CharucoDetector(
                self.charuco_board, detector_params
            )
            charuco_corners = charuco_ids = None
            charuco_corners, charuco_ids, charuco_marker_corners, charuco_marker_ids = (
                charuco_detector.detectBoard(
                    gray, charuco_corners, charuco_ids, corners, ids
                )
            )
            if charuco_corners is not None and charuco_ids is not None:
                frame = aruco.drawDetectedCornersCharuco(
                    frame, charuco_corners, charuco_ids
                )

            if charuco_ids is not None and len(charuco_ids) > 3:
                rvec = np.zeros((3, 1))
                tvec = np.zeros((3, 1))
                retval = aruco.estimatePoseCharucoBoard(
                    charuco_corners,
                    charuco_ids,
                    self.charuco_board,
                    camera_matrix,
                    dist_coeffs,
                    rvec,
                    tvec,
                )

                if retval[0]:

                    self.publish(frame, camera_name, frame_id, rvec, tvec, marker_frame)

        cv2.imshow(camera_name, frame)
        cv2.waitKey(1)

    def publish(self, pose, camera_name, frame_id, rvec, tvec, marker_frame):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
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
        if camera_name == "D405":
            self.publisher.publish(pose)
        else:
            self.D435publisher.publish(pose)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = marker_frame
        t.transform.translation.x = tvec[0][0]
        t.transform.translation.y = tvec[1][0]
        t.transform.translation.z = tvec[2][0]
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CharucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
