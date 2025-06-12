import mujoco
import numpy as np
import math
import mujoco.viewer

def get_user_input():
    # Ask for the XML model file link
    model_path = "model/mjmodel.xml"

    # Load model to get number of joints
    model = mujoco.MjModel.from_xml_path(model_path)
    num_joints = model.nq
    
    # Ask for joint angles based on model
    print(f"Enter {num_joints} joint angles in radians separated by spaces:")
    angles = list(map(float, input().split()))
    if len(angles) != num_joints:
        raise ValueError(f"Must provide exactly {num_joints} joint angles")
    
    # No conversion needed since input is already in radians
    angles = np.array(angles)
    
    return model_path, angles

def get_link_positions(model, data):
    # Get positions of all bodies/links
    positions = []
    orientations = []
    
    for i in range(model.nbody):
        # Get position of each body in world coordinates
        pos = data.xpos[i]
        # Get orientation of each body as rotation matrix
        rot = data.xmat[i].reshape(3,3)
        
        positions.append(pos)
        orientations.append(rot)
        
        print(f"\nBody {i} ({model.body(i).name}):")
        print(f"Position (x,y,z): {pos}")
        print(f"Orientation matrix:\n{rot}")
    
    return positions, orientations

def visualize_simulation(model_path, joint_angles):
    # Load the model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Disable physics simulation
    model.opt.gravity[0] = 0
    model.opt.gravity[1] = 0
    model.opt.gravity[2] = 0
    
    # Disable all actuator forces
    model.opt.disableflags = mujoco.mjtDisableBit.mjDSBL_ACTUATION
    
    # Set integration parameters to minimize movement
    model.opt.timestep = 0.0001
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4
    
    # Set joint angles and update model state
    for i, angle in enumerate(joint_angles):
        data.qpos[i] = angle
        data.qvel[i] = 0  # Set velocities to zero
        if i < len(data.ctrl):
            data.ctrl[i] = 0  # Zero out control inputs

    # Forward kinematics to update positions
    mujoco.mj_forward(model, data)
    
    # Get and print positions of all links
    print("\nLink positions and orientations:")
    positions, orientations = get_link_positions(model, data)

    # Visualization
    print("\nLaunching viewer...")
    viewer = mujoco.viewer.launch_passive(model, data)
    if viewer is not None:
        # Make sure state is properly updated before rendering
        viewer.sync()
        mujoco.mj_forward(model, data)
        # Update viewer with current state
        viewer.cam.distance = 2.0  # Set camera distance
        viewer.cam.azimuth = 45    # Set camera angle
        viewer.cam.elevation = -20  # Set camera elevation
        mujoco.viewer.launch(model, data)
        viewer.close()

def main():
    # Step 1: Get user inputs for joint angles
    model_path, joint_angles = get_user_input()
    
    # Step 2: Visualize the robot and print link positions
    visualize_simulation(model_path, joint_angles)

if __name__ == "__main__":
    main()
