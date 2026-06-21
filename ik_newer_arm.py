import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from scipy.spatial.transform import Rotation as R

from network_utils import UDP_Client, TCP_Client, TCP_Server, UDP_Server
import threading
import socket
import json

"""
l1: length of first arm
l2: length of second arm
l3: length of third arm
l4: length of fourth arm
h1: height from base of shoulder motor
h4: height of motor (dist from center of spherical wrist to end effector)
"""
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

def clamp_theta(theta):
    clamped_theta = np.copy(theta)
    for i in range(len(theta)):
        min_lim, max_lim = theta_lims[i]
        clamped_theta[i] = np.clip(clamped_theta[i], min_lim, max_lim)
    return clamped_theta

def forward_kinematics(dh_params):
    T = np.identity(4)
    transforms = [T]
    for i in range(len(dh_params)):
        a, alpha, d, theta = dh_params[i]
        T = T @ dh_transform(a, alpha, d, theta)
        transforms.append(T.copy())
    return transforms

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

    transforms = forward_kinematics(dh_params(theta))
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

        perturbed_transforms = forward_kinematics(dh_params(theta_perturb))
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

def decoupled_inverse_kinematics(theta, target_pos, target_rot, ax=None, max_iters=25, tol=1e-3, damping=0.001, run_to_completion=True, show_intermediate=True):
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

        transforms = forward_kinematics(dh_params(theta))
    
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

        if (show_intermediate or convergence_met) and ax is not None:
            ax.clear()
            ax.set_xlim(-0.8, 0.8)
            ax.set_ylim(-0.8, 0.8)
            ax.set_zlim(-0.2, 0.8)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            
            # Using transforms_current saves us a forward kinematics call!
            positions = np.array([T[:3, 3] for T in transforms_current])
            ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'ro-', linewidth=3)
            ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='blue', marker='o', s=100, label='End Effector')
            ax.scatter(target_pos[0], target_pos[1], target_pos[2], c='green', marker='x', s=100, label='Target')
            
            # end-effector orientation
            scale = 0.1
            for i in range(3):  
                ax.quiver(positions[-1, 0], positions[-1, 1], positions[-1, 2],  
                        current_rot[0, i] * scale, 
                        current_rot[1, i] * scale, 
                        current_rot[2, i] * scale, 
                        color=['r', 'g', 'b'][i], linewidth=2, arrow_length_ratio=0.3)
            
            # target orientation
            for i in range(3):  
                ax.quiver(target_pos[0], target_pos[1], target_pos[2],  
                        target_rot[0, i] * scale, 
                        target_rot[1, i] * scale, 
                        target_rot[2, i] * scale, 
                        color=['r', 'g', 'b'][i], linestyle='dashed', linewidth=2, arrow_length_ratio=0.3)
            
            # plot all the coords
            coord_frame_lines = []
            colors = ['r', 'g', 'b']  
            for i in range(6):
                for j in range(3):
                    line, = ax.plot([], [], [], colors[j], linewidth=1)
                    coord_frame_lines.append(line)

            for i in range(len(transforms_current)):
                T = transforms_current[i]
                pos = T[0:3, 3]

                for j in range(3):
                    axis = np.zeros(3)
                    axis[j] = 0.1 
                    axis_end = pos + T[0:3, j] * axis[j]

                    idx = i * 3 + j
                    if idx < len(coord_frame_lines):
                        coord_frame_lines[idx].set_data([pos[0], axis_end[0]], [pos[1], axis_end[1]])
                        coord_frame_lines[idx].set_3d_properties([pos[2], axis_end[2]])

            plt.legend()
            plt.pause(0.05)
        
        if convergence_met:
            print(f"Converged in {iter_count+1} iterations with error {error_norm:.6f}")
            break

        if not run_to_completion:
            break
    
    return theta
        


running = True

def on_close(event):
    global running
    running = False  # Stop the loop when the window is closed

plt.ion() 
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')
fig.canvas.mpl_connect('close_event', on_close)

theta = np.zeros(6)
theta[1] = np.pi/2
theta[2] = np.pi
target_pos = np.array([0.3, 0.3, 0.3])  
target_rot = R.from_euler('xyz', [0, 90, 45], degrees=True).as_matrix()  

start = time.time()

use_random_targets = True

while running:
    theta = decoupled_inverse_kinematics(theta, target_pos, target_rot, ax, run_to_completion=(use_random_targets), show_intermediate=(use_random_targets))
    
    # random target
    if use_random_targets and (time.time() - start > 3):
        target_pos = np.random.uniform([-0.3, -0.3, 0.3], [0.3, 0.3, 0.5])
        target_rot = R.from_euler('xyz', np.random.uniform([-30, -30, -30], [30, 30, 30]), degrees=True).as_matrix()
        start = time.time()
    else:
        # state = client_thread.get_state()
        # target_pos = np.array([state['x'], state['y'], state['z']])
        # target_rot = R.from_euler('xyz', [state['roll'], state['pitch'], state['yaw']], degrees=True).as_matrix()
        # start = time.time()
        pass
    
    arm_state = {
        'theta': theta.tolist(),
        'target_pos': target_pos.tolist(),
        'target_rot': target_rot.tolist(),
    }
    # server.broadcast(json.dumps(arm_state))

    fig.canvas.flush_events()

plt.show()

