import os
import cv2
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R

# Ensure directory for images exists
output_images_dir = "simulation_frames_virtual_camera"
os.makedirs(output_images_dir, exist_ok=True)

# Set simulation parameters
duration = 3  # seconds
framerate = 60  # Hz
num_frames = int(duration * framerate)

# Load the MuJoCo model
xml_file_path = "/home/paradocs/sim/sim_tek/scene.xml"  # Replace with the actual path
model = mujoco.MjModel.from_xml_path(xml_file_path)
data = mujoco.MjData(model)

# Get the joint id and qpos address for the virtual camera joint
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'virtual_camera_joint')
qposadr = model.jnt_qposadr[joint_id]

# Set up the camera to use the 'virtual_camera'
cam = mujoco.MjvCamera()
mujoco.mjv_defaultCamera(cam)
cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'virtual_camera')
cam.fixedcamid = cam_id

# Initialize the simulation
mujoco.mj_resetData(model, data)

# Set resolution
height = 1024
width = 1440

# Create a renderer
with mujoco.Renderer(model, height=height, width=width) as renderer:
    # Main simulation loop
    for i in range(num_frames):
        # Compute time
        t = i / num_frames  # Normalized time from 0 to 1

        # Adjusted camera position (moving along y-axis closer to the bone)
        cam_x = 0.0  # Move closer along x-axis
        cam_y = -0.3 + t * 0.6  # y goes from -0.3 to 0.3
        cam_z = 0.6  # Slightly above the bone
        cam_pos = np.array([cam_x, cam_y, cam_z])

        # Get bone position (assuming bone is static)
        bone_pos = np.array([0.45, 0, 0.5])  # Adjust if bone moves

        # Compute direction vector from camera to bone
        dir_vec = bone_pos - cam_pos
        dir_vec /= np.linalg.norm(dir_vec)  # Normalize

        # Compute rotation quaternion to point the camera at the bone
        a = np.array([1, 0, 0])  # Default forward vector along x-axis
        b = dir_vec  # Desired viewing direction

        # Handle potential issues with angle calculation
        cross_prod = np.cross(a, b)
        if np.linalg.norm(cross_prod) < 1e-6:
            # Vectors are parallel
            quat = np.array([1, 0, 0, 0])
        else:
            axis = cross_prod / np.linalg.norm(cross_prod)
            angle = np.arccos(np.clip(np.dot(a, b), -1.0, 1.0))
            rotvec = angle * axis
            rot = R.from_rotvec(rotvec)
            quat = rot.as_quat()  # [x, y, z, w]
            quat = np.array([quat[3], quat[0], quat[1], quat[2]])  # Convert to [w, x, y, z]

        # Set camera body position and orientation in qpos
        data.qpos[qposadr:qposadr+3] = cam_pos
        data.qpos[qposadr+3:qposadr+7] = quat

        # Step the simulation
        mujoco.mj_step(model, data)

        # Update the scene and render
        renderer.update_scene(data, camera=cam)
        pixels = renderer.render()

        # Convert pixels to BGR format for OpenCV
        frame = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)

        # Save the frame as an image
        frame_file = os.path.join(output_images_dir, f"frame_{i:04d}.png")
        cv2.imwrite(frame_file, frame)

print(f"Frames saved to {output_images_dir}")
