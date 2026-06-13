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

# theta_lims = [
#     (-np.pi, np.pi), 
#     (-4.26, 4.26),
#     (-2.36, 2.36),
#     (-3.14, 3.14),
#     (-2.36, 2.36),
#     (-2.36, 2.36)
# ]

theta_lims = [
    (-np.pi, np.pi), 
    (-np.pi/2, np.pi/2),
    (-np.deg2rad(170), np.deg2rad(170)),
    (-np.pi, np.pi),
    (-np.deg2rad(160), np.deg2rad(160)),
    (-np.deg2rad(160), np.deg2rad(160))
]

def clamp_theta(theta):
    clamped_theta = np.copy(theta)
    for i in range(len(theta)):
        min_lim, max_lim = theta_lims[i]
        clamped_theta[i] = np.clip(clamped_theta[i], min_lim, max_lim)
    return clamped_theta


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

def compute_orientation_error_5dof(R_current, R_target):
    z_current = R_current[:, 2]
    z_target = R_current[:,2]

    return np.cross(z_current, z_target)

def calculate_geometric_jacobian(theta):

    J = np.zeros((6,6))

    transforms = forward_kinematics(dh_params(theta))
    end_pos = transforms[-1][:3, 3]

    for i in range(6):
        T_prev = transforms[i]
        z_i = T_prev[:3, 2]
        p_i = T_prev[:3, 3]

        J[0:3, i] = np.cross(z_i, end_pos - p_i) # Position Jacobian

        J[3:6, i] = z_i # Orientation Jacobian

    return J, transforms[-1], transforms

def teleop_step_transpose(theta, target_pos, target_rot, ax, show_intermediate=True):
        # getting geometric jacobian + current pose
        J, T_end, transforms = calculate_geometric_jacobian(theta)
        current_pos = T_end[:3, 3]
        current_rot = T_end[:3, :3]

        # calculating errors
        pos_error = target_pos - current_pos
        ori_error = compute_orientation_error_5dof(current_rot, target_rot)
        
        # set gains (can be tuned)
        Kp_pos = 6 # for reaching (x,y,z)
        Kp_ori = 2.0 # for pointing suction cup

        error_vec = np.concatenate((pos_error * Kp_pos, ori_error * Kp_ori))

        # jacobian transpose
        delta_theta = J.T @ error_vec

        # avoiding joint limits

        q_repel = np.zeros(6)
        warning_zone = 0.35  # ~20 degrees: spring only turns on if within 20 deg of limit
        K_repel = 0.01        # strength of repelling spring
        for i in range(6):
            min_lim = theta_lims[i][0]
            max_lim = theta_lims[i][1]
        
            if theta[i] < (min_lim + warning_zone):
                # Too close to minimum! Push positive.
                q_repel[i] = K_repel * ((min_lim + warning_zone) - theta[i])
            elif theta[i] > (max_lim - warning_zone):
                # Too close to maximum! Push negative.
                q_repel[i] = -K_repel * (theta[i] - (max_lim - warning_zone))
            
        # add tracking velocities + avoidance velocities together
        # delta_theta += q_repel
        
        # clips max theta per iter to +-0.1
        delta_theta = np.clip(delta_theta, -0.2, 0.2) 
        theta += delta_theta

        # hard clamp
        theta = clamp_theta(theta)


        # --- Visual Rendering ---
        if show_intermediate:
            ax.clear()
            ax.set_xlim(-0.8, 0.8)
            ax.set_ylim(-0.8, 0.8)
            ax.set_zlim(-0.2, 0.8)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
        
            # Plot the links
            positions = np.array([T[:3, 3] for T in transforms])
            ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'ro-', linewidth=3)
            ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='blue', marker='o', s=100, label='Suction Cup')
            ax.scatter(target_pos[0], target_pos[1], target_pos[2], c='green', marker='x', s=100, label='Target')
        
            # Draw End-Effector Approach Vector (Z-Axis)
            ax.quiver(current_pos[0], current_pos[1], current_pos[2], 
                    current_rot[0, 2] * 0.1, current_rot[1, 2] * 0.1, current_rot[2, 2] * 0.1, 
                    color='b', linewidth=2, label="Current Z")
                  
            # Draw Target Approach Vector
            ax.quiver(target_pos[0], target_pos[1], target_pos[2], 
                  target_rot[0, 2] * 0.1, target_rot[1, 2] * 0.1, target_rot[2, 2] * 0.1, 
                  color='g', linestyle='dashed', linewidth=2, label="Target Z")
        
            plt.legend()
            plt.pause(0.001) # Faster pause for real-time tracking
        
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

# Setup
theta = np.zeros(6)
theta[1] = 0.5
theta[2] = 1.0
theta[3] = 0.5

start_time = time.time()
target_switch_interval = 2.0
last_update_time = time.time()
use_random_targets = True

while running:
    # 1. Update your target (from IMU UDP client or random generator)
    if use_random_targets:
        current_time = time.time()
        elapsed_time = time.time() - start_time
        radius = 0.25
        speed = 0.5 # Rads per sec
        z_height = 0.4

        # Moving in a circle while pointing slightly inward
        # target_pos = np.array([radius * np.cos(speed * elapsed_time), radius * np.sin(speed * elapsed_time), z_height])
        # target_rot = R.from_euler('xyz', [0, 90, 45], degrees=True).as_matrix()

        
        if (current_time - last_update_time) > target_switch_interval:
        # Generate a random position within your arm's workspace
        # X/Y between -0.3 and 0.3, Z between 0.2 and 0.5
            target_pos = np.random.uniform([-0.3, -0.3, 0.3], [0.3, 0.3, 0.5])
            target_rot = R.from_euler('xyz', np.random.uniform([-30, -30, -30], [30, 30, 30]), degrees=True).as_matrix()
            last_update_time = current_time # Reset the timer
        
    else:
        # Example pulling from your UDP thread:
        # state = client_thread.get_state()
        # target_pos = np.array([state['x'], state['y'], state['z']])
        # target_rot = R.from_euler('xyz', [state['roll'], state['pitch'], state['yaw']], degrees=True).as_matrix()
        pass

    # 2. Take exactly ONE step per frame toward the target
    theta = teleop_step_transpose(theta, target_pos, target_rot, ax, show_intermediate=True)

    fig.canvas.flush_events()

plt.show()
