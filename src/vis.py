import open3d as o3d

# Load the PLY file
ply_file = "/home/paradocs/sim/simulation_frames_virtual_camera/point_cloud_at_{t:.2f}.ply"  
point_cloud = o3d.io.read_point_cloud(ply_file)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud], window_name="PLY Point Cloud Visualization", width=800, height=600)
