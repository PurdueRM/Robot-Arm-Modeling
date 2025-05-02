import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Load the data
data = np.loadtxt("KFDatStatic.csv", delimiter=",", skiprows=1)

t = data[:, 0]
g = data[:, 1:4]  # ground truth
v = data[:, 4:7]  # measurement
u = data[:, 7:10] # filtered

# Setup plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3D Trajectories Over Time")

# Optionally set axis limits manually or use dynamic scaling
ax.set_xlim(np.min(data[:, [1, 4, 7]]), np.max(data[:, [1, 4, 7]]))
ax.set_ylim(np.min(data[:, [2, 5, 8]]), np.max(data[:, [2, 5, 8]]))
ax.set_zlim(np.min(data[:, [3, 6, 9]]), np.max(data[:, [3, 6, 9]]))

# Create line objects for each trajectory
g_line, = ax.plot([], [], [], 'g-', label="Ground Truth")
v_line, = ax.plot([], [], [], 'r*', label="Measurement")
u_line, = ax.plot([], [], [], 'b--', label="Filtered")

ax.legend()

# Animation update function
def update(frame):
    g_line.set_data(g[:frame+1, 0], g[:frame+1, 1])
    g_line.set_3d_properties(g[:frame+1, 2])

    v_line.set_data(v[:frame+1, 0], v[:frame+1, 1])
    v_line.set_3d_properties(v[:frame+1, 2])

    u_line.set_data(u[:frame+1, 0], u[:frame+1, 1])
    u_line.set_3d_properties(u[:frame+1, 2])

    ax.set_title(f"Time: {t[frame]:.3f} s")
    return g_line, v_line, u_line

ani = FuncAnimation(fig, update, frames=len(t), interval=50, blit=False)

plt.tight_layout()
plt.show()
