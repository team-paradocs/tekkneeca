import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_matrix
import tf2_geometry_msgs
import numpy as np
from scipy.spatial.transform import Rotation as R

def transform_to_matrix(transform):
    # transformation_matrix = []
    transformation_matrix = np.eye(4)
    trans = transform.transform.translation
    rot = transform.transform.rotation
    rot = R.from_quat([rot.x, rot.y, rot.z, rot.w])
    rot = rot.as_matrix()
    # print(rot)
    transformation_matrix[0:3, 0:3] = rot
    transformation_matrix[0:3, 3] = [trans.x, trans.y, trans.z]
    # transformation_matrix[3, :] = [0, 0, 0, 1]
    # transformation_matrix = np.array(transformation_matrix)
    return transformation_matrix


def main(node ):
    
    node = Node("tf_listener")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    for i in range(20):
        rclpy.spin_once(node)

    
    frames = tf_buffer.all_frames_as_string()
    # print(frames)
    # for frame_id in tf_buffer.all_frames_as_string().split('\n'):
        # if frame_id:
        #     print(frame_id)

    try:
        camera1_to_world = tf_buffer.lookup_transform('lbr/link_0', 'camera_color_optical_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0))
        marker2_to_camera2 = tf_buffer.lookup_transform('D435_color_optical_frame', 'marker_frame_2', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
        marker1_to_camera1 = tf_buffer.lookup_transform('camera_color_optical_frame', 'marker_1_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
        optical_frame_to_D435_link = tf_buffer.lookup_transform('D435_link', 'D435_color_optical_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
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

    # final_transform = np.linalg.inv(marker2_to_camera2_matrix) @ marker1_to_camera1_matrix @ camera1_to_world_matrix
    final_transform = camera1_to_world_matrix @ marker1_to_camera1_matrix @ np.linalg.inv(marker2_to_camera2_matrix)
    final_transform =  final_transform @ np.linalg.inv(optical_frame_to_D435_link_matrix)
    # final_transform = np.linalg.inv(final_transform)
    print(final_transform)
    
    # convert final_transform to quaternion and translation
    translation = final_transform[:3, 3]
    rotation_matrix = final_transform[:3, :3]
    r = R.from_matrix(rotation_matrix)
    quaternion = r.as_quat()
    print("Translation:", translation)
    print("Quaternion:", quaternion)

    degrees = R.from_quat(quaternion).as_euler('xyz', degrees=True)
    print("Rotation in degrees:", degrees)




        
    # except Exception as e:
    #     print("Error:", e)



    

    rclpy.shutdown()

if __name__ == "__main__":
    rclpy.init()
    node = None
    main(node)
# import rclpy
# from tf2_ros import TransformListener, Buffer
# import tf2_geometry_msgs
# from rclpy.node import Node
# import time
# import tf2_msgs.msg


# class calibrateD435(Node):
#     def __init__(self):
#         super().__init__('calibration_node')
#         # self.static_subscription = self.create_subscription(
#         #     tf2_msgs.msg.TFMessage,
#         #     '/tf_static',
#         #     self.tf_Static_callback,
#         #     10
#         # )
#         # self.subscription = self.create_subscription(
#         #     tf2_msgs.msg.TFMessage,
#         #     '/tf',
#         #     self.tf_callback,
#         #     10
#         # )


#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#         frames = self.tf_buffer.all_frames_as_string()
#         print(frames)
#         self.static_flag = False
#         # self.evaluate()
        

#     # def tf_Static_callback(self, msg):
#     #     # for transform in msg.transforms:
#     #     #     self.tf_buffer.set_transform(transform, 'default_authority')
#     #     print("Static frames:")
#     #     self.static_flag = True

#     # def tf_callback(self, msg):
#     #     # for transform in msg.transforms:
#     #     #     self.tf_buffer.set_transform(transform, 'default_authority')
#     #     print("Frames:")

#     #     if self.static_flag:

#     #         self.evaluate()        

#     # def evaluate(self):
#         # Give some time for the listener to fill the buffer
#         # time.sleep(1)

#         print("All frames:")

#         frames = self.tf_buffer.all_frames_as_string()
#         print(frames)

#         try:
#             camera1_to_world = self.tf_buffer.lookup_transform('lbr/link_0', 'camera_color_optical_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0))
#             marker2_to_camera2 = self.tf_buffer.lookup_transform('D435_color_optical_frame', 'marker_frame_2', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
#             marker1_to_camera1 = self.tf_buffer.lookup_transform('camera_color_optical_frame', 'marker_1_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))

#         except Exception as e:
#             print("transform error", e)
#             exit()
#             return

#         # Calculate the transform from camera2 to world
#         camera2_to_world = tf2_geometry_msgs.do_transform_pose(marker2_to_camera2, camera1_to_world)
#         print(camera2_to_world)


# def main(args=None):
#     rclpy.init(args=args)
#     node = calibrateD435()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
