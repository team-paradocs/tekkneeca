import rclpy

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image


import cv2
import numpy as np
from cv_bridge import CvBridge
import open3d as o3d
import open3d_conversions

import threading

ply = 0
RGB =0
depth = 0 
count=0
class SaveData(Node):

                
    def __init__(self):
        super().__init__('serial_writer')
        self.subscription=self.create_subscription(PointCloud2,
                                                   '/camera/depth/color/points',
                                                   self.ply_callback,10
        )
        self.subscription=self.create_subscription(Image,
                                                   '/camera/color/image_rect_raw',
                                                   self.rgb_callback,10
        )
        self.subscription=self.create_subscription(Image,
                                                   '/camera/depth/image_rect_raw',
                                                   self.depth_callback,10
        )
        # threading.Thread(target=self.save_files).start()
        self.input_thread = threading.Thread(target=self.wait_for_input)
        self.input_thread.daemon = True
        self.input_thread.start()
        

    def ply_callback(self, msg):
        global ply
        ply = msg
    
    def depth_callback(self, msg):
        global depth
        depth = msg

    def rgb_callback(self, msg):
        
        global RGB
        RGB = msg
        # Initialize CvBridge
        bridge = CvBridge()

        # Convert msg from ros image to opencv image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Display opencv image
        cv2.imshow('image', cv_image)
        cv2.waitKey(1)

    def wait_for_input(self):
        """Waits for the user input to register the latest received point cloud."""
        while rclpy.ok():
            input("Press Enter to save the data")
            self.save_files()

    def save_files(self):
        global ply, RGB, depth, count
        # Initialize CvBridge
        bridge = CvBridge()
        # Convert msg from ros image to opencv image
        cv_image = bridge.imgmsg_to_cv2(RGB, "bgr8")
        # save RGB image as png
        cv2.imwrite('dataset/rgb/bone'+str(count)+'_Color.png', cv_image)
        # Save depth as a raw file from Image 
        # depth_image = bridge.imgmsg_to_cv2(depth, "passthrough")
        # cv2.imwrite('dataset/depth.raw', depth_image)


        cloud = open3d_conversions.from_msg(ply)
        o3d.io.write_point_cloud("dataset/ply/bone"+str(count)+".ply", cloud)
        count = count + 1



def main(args=None):
    rclpy.init(args=args)
    saveData=SaveData()
    rclpy.spin(saveData)
    rclpy.shutdown()

if __name__=='__main__':
    main()




