import mujoco
import mujoco.viewer

# Load the MJCF model from the XML file
model = mujoco.MjModel.from_xml_path("scene.xml")

# Create a data object for the model
data = mujoco.MjData(model)

# Define the key callback function
def key_callback(keycode):
    if chr(keycode) == 'q':
        viewer.close()

# Create a viewer with the key callback
with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    # Simulation loop
    while viewer.is_running():
        # Step the simulation
        mujoco.mj_step(model, data)
        
        # Update the viewer
        viewer.sync()
        