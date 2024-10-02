import mujoco
import numpy as np
import PIL.Image
from dm_control import mujoco

import os
from IPython.display import clear_output
import numpy as np

# Graphics-related
import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from IPython.display import HTML, Image as IPImage
import PIL.Image

# For saving PLY files
from plyfile import PlyData, PlyElement
# Load the model and data
model = mujoco.MjModel.from_xml_path("/home/paradocs/sim/sim_tek/scene.xml")
data = mujoco.MjData(model)
physics = mujoco.Physics.from_xml_path("/home/paradocs/sim/sim_tek/scene.xml")

camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'virtual_camera')



# Set camera movement range (closest: 0.65(bone pos: 0.455), farthest: 1.5)
min_height = 0.65
max_height = 1.5

# Randomly select a t between min_height and max_height
# t = np.random.uniform(min_height, max_height) 
t = 1.5
print(f"Selected camera height: {t}")

# Get the joint ID for the camera joint (free or slide joint)
camera_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "virtual_camera_joint")


print(f"Camera joint position before update: {data.qpos[camera_joint_id]}")



# Update the camera's vertical position in the qpos (Z-axis position)
data.qpos[camera_joint_id] = t

# Step the simulation to apply the position change
mujoco.mj_step(model, data)

print(f"Camera joint position after update: {data.qpos[camera_joint_id]}")


# Fetch the camera resolution settings
width = physics.model.vis.global_.offwidth
height = physics.model.vis.global_.offheight

# Render RGB and depth images
pixels = physics.render(
    camera_id=camera_id,
    width=width,
    height=height
)

depth = physics.render(
    camera_id=camera_id,
    width=width,
    height=height,
    depth=True
)

# Normalize and save the depth image (just for reference, not the main goal)
depth_normalized = (depth - np.min(depth)) / (np.max(depth) - np.min(depth))
depth_image = PIL.Image.fromarray((depth_normalized * 255).astype(np.uint8), mode='L')
depth_image.save("depth_image_virtual_camera_at_065.png")

# Calculate the camera parameters (same as your original)
fovy = 58  
fovy_rad = np.deg2rad(fovy)
focal_length_y = 0.5 * height / np.tan(0.5 * fovy_rad)
focal_length_x = focal_length_y  

cx = width / 2.0
cy = height / 2.0

# Create a meshgrid for the pixel coordinates
u, v = np.meshgrid(np.arange(width), np.arange(height))

# Flip v for correct orientation
v = np.flipud(v)

# Flatten the depth array to match pixel arrays
z = depth.flatten()

# Compute the 3D coordinates (x, y, z)
x = ((u.flatten() - cx) * z) / focal_length_x
y = ((v.flatten() - cy) * z) / focal_length_y

points = np.vstack((x, y, z)).T

# Get the RGB colors and match them to the points
colors = pixels.reshape(-1, 3)

# Filter out invalid depth values
valid_indices = z > 0
points = points[valid_indices]
colors = colors[valid_indices]

# Create a structured array for the PLY file
vertex = np.array(
    [tuple(point) + tuple(color) for point, color in zip(points, colors)],
    dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
           ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
)

# Write the point cloud to a .ply file
ply_element = PlyElement.describe(vertex, 'vertex')
ply_filename = 'point_cloud_at_065.ply'
PlyData([ply_element], text=True).write(ply_filename)

print(f"Point cloud saved to {ply_filename}")
