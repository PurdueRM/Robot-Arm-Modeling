import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from scipy.spatial.transform import Rotation as R

from network_utils import UDP_Client, TCP_Client, TCP_Server, UDP_Server
import threading
import socket
import json

PORT = 65432
BROADCAST_PORT = 65433

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

def calculate_full_jacobian_numerical(theta, target_pos, target_rot,damping=0.01):
    # based on this paper: https://www.nature.com/articles/s41598-025-19054-y

    J = np.zeros((6, 6))  # 6x6 for full control

    transforms = forward_kinematics(dh_params(theta))
    current_pos = transforms[-1][:3, 3]
    current_rot = transforms[-1][:3, :3]

    weight_pos = 1.0
    weight_ori = 0.3 # np.linalg.norm(target_pos)

    # create 6x1 error vector (3 for position, 3 for orientation)
    # r_vec = np.concat([target_pos - current_pos, 
    #          1/2 * (np.cross(target_rot[:,0], current_rot[:,0]) + np.cross(target_rot[:,1], current_rot[:,1]) + np.cross(target_rot[:,2], current_rot[:,2]))])

    r_vec = np.concat([(target_pos - current_pos) * weight_pos, 
            compute_orientation_error(current_rot, target_rot) * weight_ori])


    G_theta = np.dot(r_vec, r_vec)

    gamma = 1e-6
    epsilon = 1e-9
    lambda_val = damping
    for i in range(6):
        d_i = min(abs(theta[i] - theta_lims[i][0]), abs(theta[i] - theta_lims[i][1]))
        lambda_val += gamma / (d_i**2 + epsilon)

    # Teeheehee Secret message
    # numerical Jacobian
    epsilon = 1e-6
    for i in range(6):
        theta_perturb = np.copy(theta)
        theta_perturb[i] += epsilon

        perturbed_transforms = forward_kinematics(dh_params(theta_perturb))
        perturbed_pos = perturbed_transforms[-1][:3, 3]
        perturbed_rot = perturbed_transforms[-1][:3, :3]

        J[0:3, i] = (perturbed_pos - current_pos) / epsilon
        R_err = perturbed_rot @ current_rot.T
        r = R.from_matrix(R_err)
        J[3:6, i] = r.as_rotvec() / epsilon

        # weighted jacobian
        # J = np.diag([weight_pos, weight_pos, weight_pos, weight_ori, weight_ori, weight_ori]) @ J

    # calc damping factor


    # damped pseudoinverse
    # J_pinv = np.linalg.inv(J.T @ J + damping * np.eye(6)) @ J.T
    J_pinv = np.linalg.inv(J.T @ J + lambda_val * np.eye(6)) @ J.T

    return J_pinv, current_pos, current_rot

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

def decoupled_inverse_kinematics(theta, target_pos, target_rot, ax, max_iters=25, tol=1e-3, damping=0.01, run_to_completion=True, show_intermediate=True):
    # wrist center position
    wrist_center = get_wrist_center(target_pos, target_rot)
    use_numerical_jacobian = True

    error_list = []
    error_sum = 0.0

    curr_theta = np.copy(theta)

    for iter_count in range(max_iters):
        if not running:
            break

        if use_numerical_jacobian:
            J_pinv, current_wc, current_rot = calculate_full_jacobian_numerical(theta, target_pos, target_rot,damping)

            # position error for end effector
            wc_error = target_pos - current_wc
            wc_error_norm = np.linalg.norm(wc_error)

            rot_error = compute_orientation_error(current_rot, target_rot)
            rot_error_norm = np.linalg.norm(rot_error)
            
            # update joints 1-3 using Jacobian
            delta_theta = J_pinv @ np.concatenate((wc_error, rot_error))
            delta_theta = np.clip(delta_theta, -0.2, 0.2)  # Limit step size

            # use full Jacobian to update all 6 joints as different arm config (no longer spherical joint)            
            theta += delta_theta
        else:
            # use Jacobian method for first 3 joints to reach wrist center
            raise NotImplementedError("I'm too lazy to do analytical Jacobian rn, also doesn't help that much (still numerically unstable at singularities)")
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


        # clip theta to limits
        # theta = clamp_theta(theta)

        # debug test
        # theta = np.array([0.0,0.0,0.0,0.0,0.0,0.0]) # test line for debug
        # joint_to_test = 5
        # theta[joint_to_test] = curr_theta[joint_to_test] + 0.01
        
        # full forward kinematics to check error
        transforms = forward_kinematics(dh_params(theta))
        current_pos = transforms[-1][:3, 3]
        current_rot = transforms[-1][:3, :3]
        
        # errors
        pos_error = target_pos - current_pos
        ori_error = compute_orientation_error(current_rot, target_rot)
        
        total_error = np.concatenate((pos_error, ori_error))
        error_norm = np.linalg.norm(total_error)

        error_sum += error_norm
        error_list.append(error_norm)

        convergence_met = error_norm < tol # and wc_error_norm < tol

        if np.var(error_list[-10:]) < tol/10 and len(error_list) > 10:
            print("Error has plateaued, stopping iterations.")
            convergence_met = True
        

        if show_intermediate or convergence_met:
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
            ax.scatter(target_pos[0], target_pos[1], target_pos[2], c='green', marker='o', s=100, label='Target') # could use marker x
            
            # wrist center (target and current)
            # ax.scatter(wrist_center[0], wrist_center[1], wrist_center[2], c='purple', marker='o', s=80, label='Wrist Center')
            # ax.scatter(current_wc[0], current_wc[1], current_wc[2], c='orange', marker='o', s=60, label='Current Wrist Center')
            
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

use_random_targets = False

class ClientThread:
    def __init__(self, host='127.0.0.1', port=PORT):
        self.host = host
        self.port = port
        self.running = False
        self.mutex  = threading.Lock()
        self.state_data = {
            'pitch': 90,
            'yaw': 45,
            'roll': 0,
            'x': 0.4,
            'y': 0.4,
            'z': 0.4
        }
        self.thread = None
    
    def start(self):
        print("Starting client...")
        with self.mutex:
            self.running = True

        self.thread = threading.Thread(target=self.callback)
        self.thread.daemon = True
        self.thread.start()
    
    def callback(self):
        print("Starting client thread...")
        self.client = UDP_Client(host=self.host, port=self.port, broadcast_enabled=True)
        while True:
            try:
                data = self.client.receive()
            except socket.timeout:
                data = None
            if data:
                # print(f"Received data: {data}")
                try:
                    data_dict = json.loads(data)
                    with self.mutex:
                        self.state_data['pitch'] = data_dict.get('pitch', 0.0)
                        self.state_data['yaw'] = data_dict.get('yaw', 0.0)
                        self.state_data['roll'] = data_dict.get('roll', 0.0)
                        self.state_data['x'] = data_dict.get('x', 0.0)
                        self.state_data['y'] = data_dict.get('y', 0.0)
                        self.state_data['z'] = data_dict.get('z', 0.0)
                except json.JSONDecodeError:
                    print(f"Invalid data received: {data}")

            with self.mutex:
                if not self.running:
                    break
        self.client.disconnect()
        print("Client thread stopped.")
    
    def get_state(self):
        with self.mutex:
            return self.state_data.copy()
    
    def stop(self):
        print("Stopping client thread...")
        with self.mutex:
            self.running = False
        # if self.thread:
        #     self.thread.join()

client_thread = ClientThread()
client_thread.start()

server = UDP_Server(host='127.0.0.1',port=BROADCAST_PORT, broadcast_enabled=True)
server.start()

show_intermediate = True
run_to_completion = False

while running:
    theta = decoupled_inverse_kinematics(theta, target_pos, target_rot, ax, run_to_completion=run_to_completion, show_intermediate=show_intermediate)
    
    # random target
    if use_random_targets and (time.time() - start > 2):
        target_pos = np.random.uniform([-0.3, -0.3, 0.3], [0.3, 0.3, 0.5])
        target_rot = R.from_euler('xyz', np.random.uniform([-30, -30, -30], [30, 30, 30]), degrees=True).as_matrix()
        start = time.time()
    else:
        state = client_thread.get_state()
        target_pos = np.array([state['x'], state['y'], state['z']])
        target_rot = R.from_euler('xyz', [state['roll'], state['pitch'], state['yaw']], degrees=True).as_matrix()
        start = time.time()
    
    arm_state = {
        'theta': theta.tolist(),
        'target_pos': target_pos.tolist(),
        'target_rot': target_rot.tolist(),
    }
    server.broadcast(json.dumps(arm_state))

    fig.canvas.flush_events()

plt.show()

client_thread.stop()
server.stop()