import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from scipy.spatial.transform import Rotation as R

"""
l1: length of first arm
l2: length of second arm
h1: height from base of shoulder motor
h2: offset of second arm (it is L shaped)
h3: offset of spherical wrist center
h4: height of motor (dist from center of spherical wrist to end effector)
"""
L1 = 0.4
L2 = 0.3
H1 = 0.1
H2 = 0.1
H3 = 0.05
H4 = 0.05

# DH parameters 
def dh_params(theta):
    return np.array([
        [0, np.pi/2, H1, theta[0]], # Joint 1 (base rotation)
        [L1, 0, 0, theta[1] + np.pi/2], # Joint 2 (shoulder)
        [H2, np.pi/2, 0, -theta[2] + np.pi/2], # Joint 3 (elbow)
        [0, -np.pi/2, L2 + H3, theta[3]], # Joint 4 (wrist 1)
        [0, -np.pi/2, 0, theta[4]], # Joint 5 (wrist 2)
        [0, 0, H4, theta[5]] # Joint 6 (wrist 3)
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

def compute_orientation_error(R_current, R_target):
    R_err = R_target @ R_current.T
    r = R.from_matrix(R_err)
    return r.as_rotvec()

def get_wrist_center(target_pos, target_rot):
    """
    basically since we have a spherical wrist, I just take the target orientation
    and offset the z axis by our wrist size. then we just target this point and 
    it's a 3DOF problem
    """
    
    z_axis = target_rot[:, 2]
    wrist_center = target_pos - H4 * z_axis
    
    return wrist_center

def calculate_pos_jacobian_inv_numerical(theta, damping=0.01):
    # Calculate Jacobian for the first 3 joints only (position control)
    J = np.zeros((3, 3))  # 3x3 b/c position x,y,z related to theta 1-3
    
    temp_theta = np.concatenate([theta[:3], np.zeros(3)])
    
    # current wrist center position
    transforms = forward_kinematics(dh_params(temp_theta))
    current_wc = transforms[4][:3, 3]  # Position after joint 3 (before joint 4)
    
    # numerical Jacobian
    epsilon = 1e-6
    for i in range(3):  # For each of the first 3 joints
        theta_perturb = np.copy(temp_theta)
        theta_perturb[i] += epsilon
        
        perturbed_transforms = forward_kinematics(dh_params(theta_perturb))
        perturbed_wc = perturbed_transforms[4][:3, 3]
        
        # pos part of the Jacobian
        J[:, i] = (perturbed_wc - current_wc) / epsilon
    
    # damped pseudoinverse
    J_pinv = np.linalg.inv(J.T @ J + damping * np.eye(3)) @ J.T
    
    return J_pinv, current_wc

def calculate_pos_jacobian(theta):
    """
    J11 = (h₂⋅cos(θ₂ - θ₃) + h₃⋅sin(θ₂ - θ₃) + l₁⋅sin(θ₂) + l₂⋅sin(θ₂ - θ₃))⋅sin(θ₁)
    J12 = (h₂⋅sin(θ₂ - θ₃) - h₃⋅cos(θ₂ - θ₃) - l₁⋅cos(θ₂) - l₂⋅cos(θ₂ - θ₃))⋅cos(θ₁)
    J13 = (-h₂⋅sin(θ₂ - θ₃) + h₃⋅cos(θ₂ - θ₃) + l₂⋅cos(θ₂ - θ₃))⋅cos(θ₁)
    J21 = -(h₂⋅cos(θ₂ - θ₃) + h₃⋅sin(θ₂ - θ₃) + l₁⋅sin(θ₂) + l₂⋅sin(θ₂ - θ₃))⋅cos(θ₁)
    J22 = (h₂⋅sin(θ₂ - θ₃) - h₃⋅cos(θ₂ - θ₃) - l₁⋅cos(θ₂) - l₂⋅cos(θ₂ - θ₃))⋅sin(θ₁)
    J23 = (-h₂⋅sin(θ₂ - θ₃) + h₃⋅cos(θ₂ - θ₃) + l₂⋅cos(θ₂ - θ₃))⋅sin(θ₁)
    J31 = 0
    J32 = -h₂⋅cos(θ₂ - θ₃) - h₃⋅sin(θ₂ - θ₃) - l₁⋅sin(θ₂) - l₂⋅sin(θ₂ - θ₃)
    J33 = h₂⋅cos(θ₂ - θ₃) + h₃⋅sin(θ₂ - θ₃) + l₂⋅sin(θ₂ - θ₃)
    """

    theta1, theta2, theta3 = theta[:3]
    sin_theta1 = np.sin(theta1)
    cos_theta1 = np.cos(theta1)
    sin_theta2 = np.sin(theta2)
    cos_theta2 = np.cos(theta2)
    sin_theta23 = np.sin(theta2 - theta3)
    cos_theta23 = np.cos(theta2 - theta3)

    J11 = (H2 * cos_theta23 + H3 * sin_theta23 + L1 * sin_theta2 + L2 * sin_theta23) * sin_theta1
    J12 = (H2 * sin_theta23 - H3 * cos_theta23 - L1 * cos_theta2 - L2 * cos_theta23) * cos_theta1
    J13 = (-H2 * sin_theta23 + H3 * cos_theta23 + L2 * cos_theta23) * cos_theta1

    J21 = -(H2 * cos_theta23 + H3 * sin_theta23 + L1 * sin_theta2 + L2 * sin_theta23) * cos_theta1
    J22 = (H2 * sin_theta23 - H3 * cos_theta23 - L1 * cos_theta2 - L2 * cos_theta23) * sin_theta1
    J23 = (-H2 * sin_theta23 + H3 * cos_theta23 + L2 * cos_theta23) * sin_theta1

    J31 = 0
    J32 = -H2 * cos_theta23 - H3 * sin_theta23 - L1 * sin_theta2 - L2 * sin_theta23
    J33 = H2 * cos_theta23 + H3 * sin_theta23 + L2 * sin_theta23

    J = np.array([
        [J11, J12, J13],
        [J21, J22, J23],
        [J31, J32, J33]
    ])

    return J

def calculate_pos_jacobian_inv(theta, damping=0.01):
    # temporary theta with just the first 3 joints
    temp_theta = np.concatenate([theta[:3], np.zeros(3)])
    
    # current wrist center position
    transforms = forward_kinematics(dh_params(temp_theta))
    current_wc = transforms[4][:3, 3]  # pos after joint 3 (before joint 4)
    
    # exact jacobian
    J = calculate_pos_jacobian(temp_theta[:3])
    
    # damped pseudoinverse
    J_pinv = np.linalg.inv(J.T @ J + damping * np.eye(3)) @ J.T
    
    return J_pinv, current_wc

def solve_wrist_joints(R0_3, target_rot):
    # find rotation for joint 4 to 6
    R4_6 = - R0_3.T @ target_rot
    # R4_6 = target_rot @ np.linalg.inv(R0_3)
    

    # get Euler angles from R3_6
    # our spherical wrist has roll-pitch-roll (ZY'Z'') convention
    theta4 = np.arctan2(R4_6[1, 2], R4_6[0, 2])
    theta5 = np.arccos(np.clip(R4_6[2, 2], -1.0, 1.0))
    theta6 = np.arctan2(R4_6[2, 1], -R4_6[2, 0])
    
    return np.array([theta4, theta5, theta6])

def decoupled_inverse_kinematics(theta, target_pos, target_rot, ax, max_iters=25, tol=1e-3, damping=0.01):
    # wrist center position
    wrist_center = get_wrist_center(target_pos, target_rot)
    
    for iter_count in range(max_iters):
        if not running:
            break

        # use Jacobian method for first 3 joints to reach wrist center
        J_pinv, current_wc = calculate_pos_jacobian_inv(theta, damping)
        
        # position error for wrist center
        wc_error = wrist_center - current_wc
        wc_error_norm = np.linalg.norm(wc_error)
        
        # update joints 1-3 using Jacobian
        delta_theta = J_pinv @ wc_error
        delta_theta = np.clip(delta_theta, -0.2, 0.2)  # Limit step size
        theta[:3] += delta_theta

        theta[1]  = np.clip(theta[1], -np.pi/2, np.pi/2)
        theta[2]  = np.clip(theta[2], 0, np.pi)
        
        # get orientation of first 3 joints
        temp_transforms = forward_kinematics(dh_params(theta))
        R0_3 = temp_transforms[3][:3, :3]  # rotation matrix after joint 3
        
        # solve for the wrist joints (orientation)
        theta[3:] = solve_wrist_joints(R0_3, target_rot)
        
        # full forward kinematics to check error
        transforms = forward_kinematics(dh_params(theta))
        current_pos = transforms[-1][:3, 3]
        current_rot = transforms[-1][:3, :3]
        
        # errors
        pos_error = target_pos - current_pos
        ori_error = compute_orientation_error(current_rot, target_rot)
        
        total_error = np.concatenate((pos_error, ori_error))
        error_norm = np.linalg.norm(total_error)
        
        ax.clear()
        ax.set_xlim(-0.8, 0.8)
        ax.set_ylim(-0.8, 0.8)
        ax.set_zlim(-0.2, 0.8)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        positions = np.array([T[:3, 3] for T in transforms])
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'ro-', linewidth=3)
        ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='blue', marker='o', s=100, label='End Effector')
        ax.scatter(target_pos[0], target_pos[1], target_pos[2], c='green', marker='x', s=100, label='Target')
        
        # wrist center (target and current)
        ax.scatter(wrist_center[0], wrist_center[1], wrist_center[2], c='purple', marker='o', s=80, label='Wrist Center')
        ax.scatter(current_wc[0], current_wc[1], current_wc[2], c='orange', marker='o', s=60, label='Current Wrist Center')
        
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
        colors = ['r', 'g', 'b']  # x, y, z axes 
        for i in range(6):
            for j in range(3):
                line, = ax.plot([], [], [], colors[j], linewidth=1)
                coord_frame_lines.append(line)

        for i in range(len(transforms)):
            T = transforms[i]
            pos = T[0:3, 3]

            for j in range(3):
                axis = np.zeros(3)
                axis[j] = 0.1 # random length for axis drawn idk 
                axis_end = pos + T[0:3, j] * axis[j]

                idx = i * 3 + j
                if idx < len(coord_frame_lines):
                    coord_frame_lines[idx].set_data([pos[0], axis_end[0]], [pos[1], axis_end[1]])
                    coord_frame_lines[idx].set_3d_properties([pos[2], axis_end[2]])


        plt.legend()
        plt.pause(0.05)
        
        # check convergence 
        if error_norm < tol and wc_error_norm < tol:
            print(f"Converged in {iter_count+1} iterations with error {error_norm:.6f}")
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

while running:
    theta = decoupled_inverse_kinematics(theta, target_pos, target_rot, ax)
    
    # random target
    if time.time() - start > 2:
        target_pos = np.random.uniform([-0.3, -0.3, 0.3], [0.3, 0.3, 0.5])
        target_rot = R.from_euler('xyz', np.random.uniform([-30, -30, -30], [30, 30, 30]), degrees=True).as_matrix()
        start = time.time()

    fig.canvas.flush_events()

plt.show()