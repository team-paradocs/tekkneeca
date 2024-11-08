import mujoco
import mujoco.viewer
import os
os.environ['MUJOCO_GL'] = 'egl'

import subprocess
if subprocess.run('nvidia-smi').returncode:
    raise RuntimeError(
        'Cannot communicate with GPU. '
        'Make sure you are using a GPU Colab runtime. '
        'Go to the Runtime menu and select Choose runtime type.')

# Add an ICD config so that glvnd can pick up the Nvidia EGL driver.
NVIDIA_ICD_CONFIG_PATH = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
if not os.path.exists(NVIDIA_ICD_CONFIG_PATH):
    with open(NVIDIA_ICD_CONFIG_PATH, 'w') as f:
        f.write("""{
        "file_format_version" : "1.0.0",
        "ICD" : {
            "library_path" : "libEGL_nvidia.so.0"
        }
    }
    """)



from dm_control import mujoco



#@title Other imports and helper functions


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

model = mujoco.MjModel.from_xml_path("sim_tek/scene.xml")

# Create a data object for the model
data = mujoco.MjData(model)

physics = mujoco.Physics.from_xml_path("sim_tek/scene.xml")

camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'virtual_camera')

for i in range(model.ncam):
    print(f"Camera {i}: {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)}")


width = physics.model.vis.global_.offwidth
height = physics.model.vis.global_.offheight

pixels = physics.render(
    camera_id=camera_id,
    width=width,
    height=height
)

rgb_image = PIL.Image.fromarray(pixels)
rgb_image.save("rgb_image_virtual_camera.png")
IPImage("rgb_image_virtual_camera.png") 

depth = physics.render(
    camera_id=camera_id,
    width=width,
    height=height,
    depth=True
)



depth_normalized = (depth - np.min(depth)) / (np.max(depth) - np.min(depth))
depth_image = PIL.Image.fromarray((depth_normalized * 255).astype(np.uint8), mode='L')
depth_image.save("depth_image_virtual_camera.png")
IPImage("depth_image_virtual_camera.png")  


fovy = 58  
fovy_rad = np.deg2rad(fovy)
focal_length_y = 0.5 * height / np.tan(0.5 * fovy_rad)
focal_length_x = focal_length_y  

cx = width / 2.0
cy = height / 2.0

u, v = np.meshgrid(np.arange(width), np.arange(height))


v = np.flipud(v)


z = depth.flatten()

x = ((u.flatten() - cx) * z) / focal_length_x
y = ((v.flatten() - cy) * z) / focal_length_y

points = np.vstack((x, y, z)).T

colors = pixels.reshape(-1, 3)

valid_indices = z > 0
points = points[valid_indices]
colors = colors[valid_indices]


vertex = np.array(
    [tuple(point) + tuple(color) for point, color in zip(points, colors)],
    dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
           ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
)

ply_element = PlyElement.describe(vertex, 'vertex')

ply_filename = 'point_cloud.ply'
PlyData([ply_element], text=True).write(ply_filename)

print(f"Point cloud saved to {ply_filename}")
