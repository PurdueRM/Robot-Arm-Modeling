import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation


L1 = 0.3
L2 = 0.3
L1_vert_offset = 0.1
L2_vert_offset = 0.1
L3_forward_offset = 0.1
L3_horizontal_offset = 0.1

L3 = 0.1
L4 = 0.1


# DH parameters (Denavit-Hartenberg)
# [a, alpha, d, theta]
def dh_params(theta1, theta2, theta3, theta4, theta5, theta6):
    # return np.array([
    #     [0, np.pi/2, 0.1, theta1], # Joint 1 (base rotation)
    #     [0.3, 0, 0, theta2 + np.pi/2], # Joint 2 (shoulder)
    #     [0.1, np.pi/2, 0, -theta3 + np.pi/2], # Joint 3 (elbow)
    #     [0, -np.pi/2, 0.3 + 0.1, theta4], # Joint 4 (wrist 1)
    #     [0, -np.pi/2, 0, theta5], # Joint 5 (wrist 2)
    #     [0, 0, 0.1, theta6] # Joint 6 (wrist 3)
    # ]) # old arm

    return np.array([
        [0, np.pi/2, L1_vert_offset, theta1], # Joint 1 (base rotation)
        [L1, 0, 0, theta2 + np.pi/2], # Joint 2 (shoulder)
        [0, np.pi/2, 0, -theta3 + np.pi/2], # Joint 3 (elbow)
        [0, -np.pi/2, L2, theta4], # Joint 4 (twist 1)
        [L3, 0, 0, theta5], # Joint 5 (wrist 2)
        [L4, 0, 0, theta6] # Joint 6 (wrist 3)
    ])

# transformation matrix based on DH parameters
def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# forward kinematics
def forward_kinematics(dh_params):
    T = np.identity(4)
    transforms = [T.copy()]
    for i in range(len(dh_params)):
        a, alpha, d, theta = dh_params[i]
        T = T @ dh_transform(a, alpha, d, theta)
        transforms.append(T.copy())
    return transforms

# extract joint positions from transformation matrices
def get_joint_positions(transforms):
    positions = np.zeros((len(transforms), 3))
    for i in range(len(transforms)):
        positions[i] = transforms[i][0:3, 3]
    return positions

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
theta5 = 0
theta6 = 0

slider_ax1 = plt.axes([0.25, 0.02, 0.65, 0.03])
slider_ax2 = plt.axes([0.25, 0.06, 0.65, 0.03])
slider_ax3 = plt.axes([0.25, 0.10, 0.65, 0.03])
slider_ax4 = plt.axes([0.25, 0.14, 0.65, 0.03])
slider_ax5 = plt.axes([0.25, 0.18, 0.65, 0.03])
slider_ax6 = plt.axes([0.25, 0.22, 0.65, 0.03])

slider1 = Slider(slider_ax1, 'Joint 1', -np.pi, np.pi, valinit=theta1)
slider2 = Slider(slider_ax2, 'Joint 2', -np.pi, np.pi, valinit=theta2)
slider3 = Slider(slider_ax3, 'Joint 3', -np.pi, np.pi, valinit=theta3)
slider4 = Slider(slider_ax4, 'Joint 4', -np.pi, np.pi, valinit=theta4)
slider5 = Slider(slider_ax5, 'Joint 5', -np.pi, np.pi, valinit=theta5)
slider6 = Slider(slider_ax6, 'Joint 6', -np.pi, np.pi, valinit=theta6)

arm_points, = ax.plot([], [], [], 'ro-', linewidth=3, markersize=8)
end_effector = ax.scatter([], [], [], c='blue', marker='o', s=100)

# coordinate frames
coord_frame_lines = []
colors = ['r', 'g', 'b']  # x, y, z axes colors
for i in range(6):
    for j in range(3):  # 3 axes per frame
        line, = ax.plot([], [], [], colors[j], linewidth=1)
        coord_frame_lines.append(line)

ax.set_xlim(-0.8, 0.8)
ax.set_ylim(-0.8, 0.8)
ax.set_zlim(-0.2, 0.8)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('6-DoF Robot Arm Visualization')

def update_plot(val=None):
    theta1 = slider1.val
    theta2 = slider2.val
    theta3 = slider3.val
    theta4 = slider4.val
    theta5 = slider5.val
    theta6 = slider6.val

    dh = dh_params(theta1, theta2, theta3, theta4, theta5, theta6)
    transforms = forward_kinematics(dh)

    # joint positions
    positions = get_joint_positions(transforms)

    # updat arm line plot
    arm_points.set_data(positions[:, 0], positions[:, 1])
    arm_points.set_3d_properties(positions[:, 2])

    # end effector position
    end_effector._offsets3d = ([positions[-1, 0]], [positions[-1, 1]], [positions[-1, 2]])

    # joint coordinate frames
    for i in range(len(transforms)):
        T = transforms[i]
        pos = T[0:3, 3]

        # draw each axis (x, y, z) 
        for j in range(3):
            axis = np.zeros(3)
            axis[j] = 0.1  # axis length
            axis_end = pos + T[0:3, j] * axis[j]

            idx = i * 3 + j
            if idx < len(coord_frame_lines):
                coord_frame_lines[idx].set_data([pos[0], axis_end[0]], [pos[1], axis_end[1]])
                coord_frame_lines[idx].set_3d_properties([pos[2], axis_end[2]])

    fig.canvas.draw_idle()

slider1.on_changed(update_plot)
slider2.on_changed(update_plot)
slider3.on_changed(update_plot)
slider4.on_changed(update_plot)
slider5.on_changed(update_plot)
slider6.on_changed(update_plot)

plt.subplots_adjust(bottom=0.3)
plt.show()
