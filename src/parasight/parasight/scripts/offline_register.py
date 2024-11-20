import numpy as np
import open3d as o3d
import cv2
import os

from ament_index_python.packages import get_package_share_directory

from parasight.sam_ui import SegmentAnythingUI
from parasight.registration import RegistrationPipeline
from parasight.utils import *


def load_data(idx, data_dir="/ros_ws/src/data"):
    package_dir = get_package_share_directory('parasight')
    rgb = cv2.cvtColor(cv2.imread(os.path.join(data_dir, f"rgb_{idx:04d}.png")), cv2.COLOR_BGR2RGB)
    with np.load(os.path.join(data_dir, f"depth_{idx:04d}.npz")) as data:
        depth = data['depth'].astype(np.float32) / 1000.0
    pcd = o3d.io.read_point_cloud(os.path.join(data_dir, f"cloud_{idx:04d}.ply"))
    source_pcd = o3d.io.read_point_cloud(os.path.join(package_dir, "resource", "femur_shell.ply"))
    return rgb, depth, pcd, source_pcd

def add_depth(points, depth_image, kernel_size=5):
    new_points = []
    h, w = depth_image.shape
    half_kernel = kernel_size // 2

    for x, y in points:
        y, x = int(y), int(x)
        y_start, y_end = max(0, y - half_kernel), min(h, y + half_kernel + 1)
        x_start, x_end = max(0, x - half_kernel), min(w, x + half_kernel + 1)
        depth = np.mean(depth_image[y_start:y_end, x_start:x_end])
        new_points.append([x, y, depth])

    return new_points

def main():
    idx = 0000
    rgb, depth, raw_pcd, source_pcd = load_data(idx)

    sam_ui = SegmentAnythingUI()
    reg_pipeline = RegistrationPipeline()


    mask, annotated_points, mask_points = sam_ui.segment_using_ui(rgb)
    annotated_points = add_depth(annotated_points, depth)
    mask_points = add_depth(mask_points, depth)

    transform, fitness = reg_pipeline.register(mask, source_pcd, raw_pcd, annotated_points, mask_points)
    source_pcd = source_pcd.voxel_down_sample(voxel_size=0.003)
    source_pcd.transform(transform)
    source_pcd.paint_uniform_color([1, 0, 0])



    o3d.visualization.draw_geometries([raw_pcd, source_pcd])

if __name__ == "__main__":
    main()