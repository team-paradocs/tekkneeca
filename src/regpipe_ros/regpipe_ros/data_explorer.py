import open3d as o3d
import os
import sys
import json
# import dataset_location
import numpy as np

class PointCloudVisualizer:
    def __init__(self, directory,markers=[]):
        self.directory = directory
        self.ply_files = [file for file in os.listdir(directory) if file.endswith('.ply')]
        if not self.ply_files:
            print("No .ply files found in the directory.")
            sys.exit(0)
        self.current_index = 0
        self.markers = markers
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        
    def load_point_cloud(self, index):
        file_path = os.path.join(self.directory, self.ply_files[index])
        return o3d.io.read_point_cloud(file_path)
    
    def update_visualization(self):
        print("Current Index: ", self.current_index)
        self.vis.clear_geometries()
        self.point_cloud = self.load_point_cloud(self.current_index)

        # Shift the point cloud to the origin
        self.point_cloud = self.point_cloud.translate(-self.point_cloud.get_center())

        # Custom Markers
        # Oriented Bounding Box
        if 'obb' in self.markers:
            obb = self.point_cloud.get_oriented_bounding_box()
            obb.color = (1, 0, 0)
            self.vis.add_geometry(obb)

        # Point Cloud Center
        if 'pcd_center' in self.markers:
            center = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
            center.compute_vertex_normals()
            center.paint_uniform_color([0, 0, 1])
            center.translate(self.point_cloud.get_center())
            self.vis.add_geometry(center)

        # cl,ind = self.point_cloud.remove_statistical_outlier(nb_neighbors=10,std_ratio=0.2)
        # total_points = np.asarray(self.point_cloud.points).shape[0]
        # print(f"Removed {total_points - np.asarray(cl.points).shape[0]} outliers from the point cloud")

        # self.point_cloud = self.point_cloud.select_by_index(ind)

        # cl,ind = self.point_cloud.remove_radius_outlier(nb_points=100,radius=0.01)
        # total_points = np.asarray(self.point_cloud.points).shape[0]
        # print(f"Removed {total_points - np.asarray(cl.points).shape[0]} outliers from the point cloud")

        # self.point_cloud = self.point_cloud.select_by_index(ind)


        

        # Add a Sphere to the point with maximum Z value
        max_z_index = np.argmax(np.asarray(self.point_cloud.points)[:,2])
        max_z_point = np.asarray(self.point_cloud.points)[max_z_index]
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
        sphere.compute_vertex_normals()
        sphere.paint_uniform_color([0, 1, 0])

        sphere.translate(max_z_point)
        self.vis.add_geometry(sphere)


        self.vis.add_geometry(self.point_cloud)
        self.vis.update_renderer()

    
    def next_cloud(self):
        self.current_index = (self.current_index + 1) % len(self.ply_files)
        self.update_visualization()
    
    def previous_cloud(self):
        self.current_index = (self.current_index - 1) % len(self.ply_files)
        self.update_visualization()

    
    def run(self):
        self.vis.create_window()
        self.vis.register_key_callback(262, lambda vis: self.next_cloud())  # Right arrow key
        self.vis.register_key_callback(263, lambda vis: self.previous_cloud())  # Left arrow key

        self.update_visualization()
        self.vis.run()

if __name__ == "__main__":
    directory_path = 'dataset/ply'
    # directory_path = "filtered"
    # markers = ['obb','pcd_center']
    markers = []
    visualizer = PointCloudVisualizer(directory_path,markers)
    visualizer.run()
