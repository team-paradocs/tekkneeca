# Description: This script is used to calibrate the D435 camera with respect to the robot base frame.
# Using the poses from the charuco_pose_publisher
import numpy as np
import rclpy
import tf2_geometry_msgs
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix


def transform_to_matrix(transform):
    """
    Converts a ROS Transform message into a 4x4 transformation matrix.

    Args:
        transform: The ROS Transform message containing translation and rotation data.

    Returns:
        A 4x4 numpy array representing the transformation matrix, where the upper-left 3x3 submatrix
        is the rotation matrix, and the last column represents the translation vector.
    """

    transformation_matrix = np.eye(4)
    trans = transform.transform.translation
    rot = transform.transform.rotation
    rot = R.from_quat([rot.x, rot.y, rot.z, rot.w])
    rot = rot.as_matrix()
    transformation_matrix[0:3, 0:3] = rot
    transformation_matrix[0:3, 3] = [trans.x, trans.y, trans.z]
    return transformation_matrix


def main(node):
    """
    Main function that initializes the ROS node, retrieves transformations for the camera
    and markers, computes the calibration transformation, and outputs the translation,
    quaternion, and rotation in degrees.

    Args:
        node: The ROS node instance.

    Workflow:
        1. Initializes the TF listener and buffer to listen for transforms.
        2. Retrieves the necessary transforms.
        3. Converts the transforms to transformation matrices.
        4. Calculates the final calibration transform.
        5. Outputs the translation, quaternion, and rotation in degrees.
    """
    node = Node("tf_listener")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    for i in range(20):
        rclpy.spin_once(node)

    frames = tf_buffer.all_frames_as_string()

    try:
        camera1_to_world = tf_buffer.lookup_transform(
            "lbr/link_0",
            "camera_color_optical_frame",
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=2.0),
        )
        marker2_to_camera2 = tf_buffer.lookup_transform(
            "D435_color_optical_frame",
            "marker_frame_2",
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=1.0),
        )
        marker1_to_camera1 = tf_buffer.lookup_transform(
            "camera_color_optical_frame",
            "marker_1_frame",
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=1.0),
        )
        optical_frame_to_D435_link = tf_buffer.lookup_transform(
            "D435_link",
            "D435_color_optical_frame",
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=1.0),
        )
    except Exception as e:
        print("transform error", e)
        main(node)
        return

    # convert to transformation matrix
    camera1_to_world_matrix = transform_to_matrix(camera1_to_world)
    marker2_to_camera2_matrix = transform_to_matrix(marker2_to_camera2)
    marker1_to_camera1_matrix = transform_to_matrix(marker1_to_camera1)
    optical_frame_to_D435_link_matrix = transform_to_matrix(optical_frame_to_D435_link)

    # convert to numpy array
    camera1_to_world_matrix = np.array(camera1_to_world_matrix)
    marker2_to_camera2_matrix = np.array(marker2_to_camera2_matrix)
    marker1_to_camera1_matrix = np.array(marker1_to_camera1_matrix)
    optical_frame_to_D435_link_matrix = np.array(optical_frame_to_D435_link_matrix)

    final_transform = (
        camera1_to_world_matrix
        @ marker1_to_camera1_matrix
        @ np.linalg.inv(marker2_to_camera2_matrix)
    )
    final_transform = final_transform @ np.linalg.inv(optical_frame_to_D435_link_matrix)
    print(final_transform)

    # convert final_transform to quaternion and translation
    translation = final_transform[:3, 3]
    rotation_matrix = final_transform[:3, :3]
    r = R.from_matrix(rotation_matrix)
    quaternion = r.as_quat()
    print("Translation:", translation)
    print("Quaternion:", quaternion)

    degrees = R.from_quat(quaternion).as_euler("xyz", degrees=True)
    print("Rotation in degrees:", degrees)

    rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    node = None
    main(node)
