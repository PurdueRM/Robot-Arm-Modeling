import mujoco
import mujoco.viewer
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

"""
l1: length of first arm
l2: length of second arm
l3: length of third arm
l4: length of fourth arm
h1: height from base of shoulder motor
h4: height of motor (dist from center of spherical wrist to end effector)
"""

'''
L1 = 0.37
L2 = 0.31
L3 = 0.11
L4 = 0.13
H1 = 0.07
H4 = 0.05

H2 = 0.1
H3 = 0.05

theta_lims = [
    (-np.pi, np.pi), 
    (-np.pi/2, np.pi/2),
(-np.deg2rad(170), np.deg2rad(170)),
    (-np.pi, np.pi),
    (-np.deg2rad(160), np.deg2rad(160)),
    (-np.deg2rad(160), np.deg2rad(160))
]

# DH parameters 
def dh_params(theta):
    return np.array([
        [0, np.pi/2, H1, theta[0]], # Joint 1 (base rotation)
        [L1, 0, 0, theta[1] + np.pi/2], # Joint 2 (shoulder)
        [0, np.pi/2, 0, -theta[2] + np.pi/2], # Joint 3 (elbow)
        [0, -np.pi/2, L2, theta[3]], # Joint 4 (twist 1)
        [L3, 0, 0, theta[4]], # Joint 5 (wrist 2)
        [L4, 0, 0, theta[5]] # Joint 6 (wrist 3)
    ])

def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(dh_params):
    T = np.identity(4)
    transforms = [T]
    for i in range(len(dh_params)):
        a, alpha, d, theta = dh_params[i]
        T = T @ dh_transform(a, alpha, d, theta)
        transforms.append(T.copy())
    return transforms
'''

URDF_OFFSETS = [
    [0.0,      0.0,     0.0,       0.0,      0.0,      0.0],       # joint5 (ground -> base)
    [0.0,     -0.0475,  0.05,     -1.5708,  -0.87031,  0.0],       # joint0 (base -> arm1)
    [0.37,     0.0,     0.075,     0.0,      0.0,      1.2473],    # joint1 (arm1 -> arm2)
    [0.185,    0.0,    -0.035,     0.0,      1.5708,   0.0],       # joint2 (arm2 -> wrist1)
    [-0.0255,  0.0,     0.125,     1.5708,  -1.3193,  -1.5708],    # joint3 (wrist1 -> hand)
    [0.1068,   0.0,    -0.047,     3.1416,   0.0,      0.0]        # joint4 (hand -> finger)
]

theta_lims = [
    (-4.26, 4.26),  # joint5 (ground -> base)
    (-4.26, 4.26),  # joint0 (base -> arm1)
    (-2.36, 2.36),  # joint1 (arm1 -> arm2)
    (-3.14, 3.14),  # joint2 (arm2 -> wrist1)
    (-2.36, 2.36),  # joint3 (wrist1 -> hand)
    (-2.36, 2.36)   # joint4 (hand -> finger)
]

def create_transform_matrix(xyz, rpy, joint_angle):
    T = np.eye(4)
    T[:3, 3] = xyz
    T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
    
    # The joint revolves around its local Z-axis
    R_joint = np.eye(4)
    R_joint[:3, :3] = R.from_euler('z', joint_angle).as_matrix()
    
    return T @ R_joint

def forward_kinematics(theta):
    T = np.identity(4)
    transforms = [T]
    
    for i in range(6):
        xyz = URDF_OFFSETS[i][:3]
        rpy = URDF_OFFSETS[i][3:]
        
        T = T @ create_transform_matrix(xyz, rpy, theta[i])
        transforms.append(T.copy())
        
    return transforms

def clamp_theta(theta):
    clamped_theta = np.copy(theta)
    for i in range(len(theta)):
        min_lim, max_lim = theta_lims[i]
        clamped_theta[i] = np.clip(clamped_theta[i], min_lim, max_lim)
    return clamped_theta

def compute_orientation_error(R_current, R_target):
    # z_current = R_current[:, 2]
    # z_target = R_target[:, 2]

    R_err = R_target @ R_current.T
    r = R.from_matrix(R_err)
    # return np.cross(z_current, z_target)

    return r.as_rotvec()

def calculate_5d_jacobian_numerical(theta, target_pos, target_rot, damping = 0.001):
    # 5x6 jacobian (omitting end-effector yaw)
    J = np.zeros((5, 6))

    transforms = forward_kinematics(theta)
    current_pos = transforms[-1][:3, 3]
    current_rot = transforms[-1][:3, :3]

    y_c = current_rot[:, 1]
    z_c = current_rot[:, 2]

    weight_pos = 1.0
    weight_ori = 0.5

    gamma = 1e-6
    epsilon = 1e-9
    lambda_val = damping
    for i in range(6):
       d_i = min(abs(theta[i] - theta_lims[i][0]), abs(theta[i] - theta_lims[i][1]))
       lambda_val += gamma / (d_i**2 + epsilon)

    # numerical Jacobian
    epsilon = 1e-6
    for i in range(6):
        theta_perturb = np.copy(theta)
        theta_perturb[i] += epsilon

        perturbed_transforms = forward_kinematics(theta_perturb)
        perturbed_pos = perturbed_transforms[-1][:3, 3]
        perturbed_rot = perturbed_transforms[-1][:3, :3]
 
        J[0:3, i] = ((perturbed_pos - current_pos) / epsilon) * weight_pos
        R_err = perturbed_rot @ current_rot.T
        omega = R.from_matrix(R_err).as_rotvec() / epsilon

        J[3, i] = np.dot(y_c, omega) * weight_ori
        J[4, i] = np.dot(z_c, omega) * weight_ori

    # damped pseudoinverse
    J_pinv = np.linalg.inv(J.T @ J + lambda_val * np.eye(6)) @ J.T

    # nullspace projection
    N = np.eye(6) - J_pinv @ J

    return J_pinv, N, transforms

def decoupled_inverse_kinematics(theta, target_pos, target_rot, model, data, viewer, qpos_idcs, max_iters=50, tol=1e-3, damping=0.001):
    error_list = []
    error_sum = 0.0

    for iter_count in range(max_iters):
        if not running:
            break

        J_pinv, N, transforms_current = calculate_5d_jacobian_numerical(theta, target_pos, target_rot, damping)

        current_pos = transforms_current[-1][:3, 3]
        current_rot = transforms_current[-1][:3, :3]
        current_wc = transforms_current[4][:3, 3]

        weight_pos = 1.0
        weight_ori = 0.5

        pos_error = target_pos - current_pos

        max_pos_step = 0.05
        pos_norm = np.linalg.norm(pos_error)
        if pos_norm > max_pos_step:
            pos_error = pos_error * (max_pos_step / pos_norm)

        rot_error_vec = compute_orientation_error(current_rot, target_rot)

        max_rot_step = 0.05
        rot_norm = np.linalg.norm(rot_error_vec)
        if rot_norm > max_rot_step:
            rot_error_vec = rot_error_vec * (max_rot_step / rot_norm)

        # projecting rot error onto local X and Y
        y_c = current_rot[:, 1]
        z_c = current_rot[:, 2]
        rot_error = np.array([np.dot(y_c, rot_error_vec), np.dot(z_c, rot_error_vec)])
        
        error_5d = np.concatenate([pos_error * weight_pos, rot_error * weight_ori])

        # joint limit avoidance

        q_null = np.zeros(6)
        k_null = 0.05 # gain for limit avoidance

        for i in range(6):
            min_lim, max_lim = theta_lims[i]
            mid_point = (max_lim + min_lim) / 2.0
            range_lim = (max_lim - min_lim)
                
            # gradient pushing the joint towards its midpoint, spikes exponentially as it gets closer to the limits.
            q_null[i] = -k_null * (theta[i] - mid_point) / ((range_lim / 2.0) ** 2)

        # combine
        delta_theta_primary = J_pinv @ error_5d
        delta_theta_null = N @ q_null

        delta_theta = delta_theta_primary + delta_theta_null
        delta_theta = np.clip(delta_theta, -0.1, 0.1)

        theta += delta_theta
        # theta = clamp_theta(theta)

        for i, qpos_idx in enumerate(qpos_idcs):
            data.qpos[qpos_idx] = theta[i]
            
        
        with viewer.lock():
            for i, qpos_idx in enumerate(qpos_idcs):
                data.qpos[qpos_idx] = theta[i]
            mujoco.mj_forward(model, data)
            
        viewer.sync()
        time.sleep(0.05)

        transforms = forward_kinematics(theta)
    
        current_pos = transforms[-1][:3, 3]
        current_rot = transforms[-1][:3, :3]

        # errors

        pos_error = target_pos - current_pos
        ori_error_raw = compute_orientation_error(current_rot, target_rot)

        y_c = current_rot[:, 1]
        z_c = current_rot[:, 2]
        ori_error_5d = np.array([np.dot(y_c, ori_error_raw), np.dot(z_c, ori_error_raw)])

        total_error_5d = np.concatenate((pos_error, ori_error_5d))
        error_norm = np.linalg.norm(total_error_5d)

        error_sum += error_norm
        error_list.append(error_norm)

        convergence_met = error_norm < tol

        if len(error_list) > 10:
            improvement = abs(error_list[-10] - error_list[-1])
            if improvement < 1e-5:
                convergence_met = True        

        if convergence_met:
            print(f"Converged in {iter_count+1} iterations with error {error_norm:.6f}")
            break

    return theta
        


running = True

def on_close(event):
    global running
    running = False  # Stop the loop when the window is closed

def main():
    model = mujoco.MjModel.from_xml_path("arm/mujoco_arm/mjmodel.xml")
    data = mujoco.MjData(model)

    arm_joint_names = ["joint5", "joint0", "joint1", "joint2", "joint3", "joint4"]
    arm_joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in arm_joint_names]
    qpos_idcs = [model.jnt_qposadr[jid] for jid in arm_joint_ids]

    target_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "target_joint")
    target_qpos_adr = model.jnt_qposadr[target_joint_id]

    current_theta = np.zeros(6)
    current_theta[1] = np.pi/2
    current_theta[2] = np.pi

    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_update_time = time.time()

        while viewer.is_running():
            current_time = time.time()

            # Generate a new target every 4 seconds to give it time to animate
            if current_time - last_update_time > 4.0:
                
                target_pos = np.random.uniform([-0.3, -0.3, 0.3], [0.3, 0.3, 0.5])
                target_rot = R.from_euler('xyz', np.random.uniform([-30, -30, -30], [30, 30, 30]), degrees=True).as_matrix()

                print(f"\nNew Target -> Pos: {np.round(target_pos, 2)}")

                # Update target sphere location
                data.qpos[target_qpos_adr : target_qpos_adr + 3] = target_pos
                scipy_quat = R.from_matrix(target_rot).as_quat()
                mujoco_quat = np.array([scipy_quat[3], scipy_quat[0], scipy_quat[1], scipy_quat[2]])
                data.qpos[target_qpos_adr + 3 : target_qpos_adr + 7] = mujoco_quat
                
                # --- RUN IK (Which now contains the viewer update loop) ---
                current_theta = decoupled_inverse_kinematics(
                    theta=current_theta.copy(), 
                    target_pos=target_pos, 
                    target_rot=target_rot,
                    model=model,
                    data=data,
                    viewer=viewer,
                    qpos_idcs=qpos_idcs,
                    max_iters=50
                )
                
                last_update_time = time.time() # Reset time *after* it finishes converging

            # Ensure the viewer stays synced while waiting for the next target
            viewer.sync()
            time.sleep(0.01)

if __name__ == "__main__":
    main()
