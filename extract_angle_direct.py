import mujoco
import numpy as np
import math
import mujoco.viewer

def get_user_input():
    # Ask for the XML model file link
    model_path = "model/mjmodel.xml"

    # Get palm target position and orientation
    print("Enter target position and orientation for palm (rh_palm):")
    x, y, z = map(float, input("Enter target position (x, y, z) in meters separated by space: ").split())
    qw, qx, qy, qz = map(float, input("Enter target orientation as quaternion (qw, qx, qy, qz): ").split())
    palm_target = {'pos': np.array([x, y, z]), 'quat': np.array([qw, qx, qy, qz])}

    # Get fingertip targets relative to palm
    fingertips = ['ff', 'mf', 'rf', 'lf', 'th']
    tip_targets = {}
    print("\nEnter fingertip positions relative to palm:")
    for tip in fingertips:
        print(f"\nTarget for rh_{tip}tip:")
        x, y, z = map(float, input(f"Enter position (x, y, z) relative to palm: ").split())
        tip_targets[f'rh_{tip}tip'] = np.array([x, y, z])

    return model_path, palm_target, tip_targets

def find_closest_valid_pose(model, data, palm_target, tip_targets):
    # Initialize variables to store best solution
    best_angles = np.zeros(model.nq)
    min_error = float('inf')
    
    # Simple IK solver with constraints
    for _ in range(100):  # Number of random trials
        # Generate random joint angles within limits
        test_angles = np.random.uniform(model.jnt_range[:,0], model.jnt_range[:,1])
        
        # Apply angles
        data.qpos[:] = test_angles
        mujoco.mj_forward(model, data)
        
        # Get palm pose
        palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rh_palm")
        current_palm_pos = data.xpos[palm_id]
        current_palm_quat = data.xquat[palm_id]
        
        # Calculate palm error
        pos_error = np.linalg.norm(current_palm_pos - palm_target['pos'])
        quat_error = np.linalg.norm(current_palm_quat - palm_target['quat'])
        
        # Calculate fingertip errors
        tip_error = 0
        for tip_name, target_pos in tip_targets.items():
            tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, tip_name)
            current_tip_pos = data.xpos[tip_id]
            # Transform to palm space
            tip_error += np.linalg.norm(current_tip_pos - target_pos)
        
        total_error = pos_error + quat_error + tip_error
        
        if total_error < min_error:
            min_error = total_error
            best_angles = test_angles.copy()
            best_palm_pos = current_palm_pos.copy()
            best_palm_quat = current_palm_quat.copy()
    
    print("\nFound closest achievable pose:")
    print(f"Palm position: {best_palm_pos}")
    print(f"Palm orientation (quaternion): {best_palm_quat}")
    
    return best_angles

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
    
    # Set joint angles
    for i, angle in enumerate(joint_angles):
        data.qpos[i] = angle
        data.qvel[i] = 0  # Set velocities to zero
        if i < len(data.ctrl):
            data.ctrl[i] = 0  # Zero out control inputs

    # Forward kinematics to update positions
    mujoco.mj_forward(model, data)

    # Visualization
    print("\nLaunching viewer...")
    viewer = mujoco.viewer.launch_passive(model, data)
    if viewer is not None:
        viewer.sync()
        mujoco.mj_forward(model, data)
        mujoco.viewer.launch(model, data)
        viewer.close()

def main():
    # Step 1: Get user inputs for target poses
    model_path, palm_target, tip_targets = get_user_input()
    
    # Step 2: Load model and find closest valid pose
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    joint_angles = find_closest_valid_pose(model, data, palm_target, tip_targets)
    
    # Step 3: Visualize the result
    visualize_simulation(model_path, joint_angles)
    
    print("\nFinal joint angles:", joint_angles)

if __name__ == "__main__":
    main()
