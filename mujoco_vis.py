import mujoco
import mujoco.viewer
import numpy as np
import time
from network_utils import UDP_Client
import json
import threading
import socket
import numpy as np
import scipy.spatial.transform as R

PORT = 65433
joint_offsets = [-np.pi, -0.71, -1.25, np.pi, -1.29, 0]
joint_inversions = [1, 1, -1, 1, -1, 1]

joint_max_vel = [3.0, 3.0, 3.0, 6.0, 6.0, 6.0] 
joint_max_accel = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]

joint_pd_gains = [
    {'kp': 100.0, 'kd': 10.0},
    {'kp': 100.0, 'kd': 10.0},
    {'kp': 80.0, 'kd': 8.0},
    {'kp': 60.0, 'kd': 6.0},
    {'kp': 40.0, 'kd': 4.0},
    {'kp': 20.0, 'kd': 2.0},
]

# model = mujoco.MjModel.from_xml_path("arm/mujoco_arm/arm.urdf")
model = mujoco.MjModel.from_xml_path("arm/mujoco_arm/mjmodel.xml")
data = mujoco.MjData(model)

print("Joints:")
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    print(i, name)

def set_joint(name, value):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    qpos_addr = model.jnt_qposadr[joint_id]
    data.qpos[qpos_addr] = value

def get_joint(name):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    qpos_addr = model.jnt_qposadr[joint_id]
    return data.qpos[qpos_addr]

def get_closest_angle(target, current):
    # Wrap target angle to [-pi, pi]
    target = (target + np.pi) % (2 * np.pi) - np.pi
    current = (current + np.pi) % (2 * np.pi) - np.pi
    
    # Calculate the difference
    diff = target - current
    
    # Wrap the difference to [-pi, pi]
    if diff > np.pi:
        diff -= 2 * np.pi
    elif diff < -np.pi:
        diff += 2 * np.pi
    
    return current + diff

def rot_mat_to_quat(mat):
    # Convert a 3x3 rotation matrix to a quaternion (w, x, y, z)
    m = mat
    tr = m[0,0] + m[1,1] + m[2,2]
    if tr > 0:
        S = np.sqrt(tr+1.0) * 2
        w = 0.25 * S
        x = (m[2,1] - m[1,2]) / S
        y = (m[0,2] - m[2,0]) / S
        z = (m[1,0] - m[0,1]) / S
    elif (m[0,0] > m[1,1]) and (m[0,0] > m[2,2]):
        S = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2
        w = (m[2,1] - m[1,2]) / S
        x = 0.25 * S
        y = (m[0,1] + m[1,0]) / S
        z = (m[0,2] + m[2,0]) / S
    elif m[1,1] > m[2,2]:
        S = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2
        w = (m[0,2] - m[2,0]) / S
        x = (m[0,1] + m[1,0]) / S
        y = 0.25 * S
        z = (m[1,2] + m[2,1]) / S
    else:
        S = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2
        w = (m[1,0] - m[0,1]) / S
        x = (m[0,2] + m[2,0]) / S
        y = (m[1,2] + m[2,1]) / S
        z = 0.25 * S
    return np.array([w, x, y, z])

class MujocoClientThread:
    def __init__(self, host='127.0.0.1', port=PORT):
        self.host = host
        self.port = port
        self.running = False
        self.mutex  = threading.Lock()
        self.state_data = {
            'theta': [0.0]*6,
            'target_pos': [0.0, 0.0, 0.0],
            'target_rot': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
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
                    # print(f"Parsed data: {data_dict}")
                    with self.mutex:
                        self.state_data['theta'] = data_dict.get('theta', self.state_data['theta'])
                        self.state_data['target_pos'] = data_dict.get('target_pos', self.state_data['target_pos'])
                        self.state_data['target_rot'] = data_dict.get('target_rot', self.state_data['target_rot'])
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

client_thread = MujocoClientThread()
client_thread.start()

target_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
joint_errors = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

with mujoco.viewer.launch_passive(model, data) as viewer:
    t = 0.0
    
    while viewer.is_running():
        # test animate joints
        # t += 0.01
        # set_joint("joint5", np.sin(t))
        # set_joint("joint0", 0.5*np.sin(t))
        # set_joint("joint1", 0.5*np.cos(t))
        # set_joint("joint2", np.sin(t*0.5))
        # set_joint("joint3", np.cos(t*0.8))
        # set_joint("joint4", 0.3*np.sin(t*2))

        state = client_thread.get_state()
        thetas = state['theta']
        joint_order = [5, 0, 1, 2, 3, 4] 

        # for i in range(6):
        #     target_joint_angles[i] = thetas[i] * joint_inversions[i] + joint_offsets[i]
        #     curr_joint_angle = get_joint(f"joint{joint_order[i]}")
        #     target_joint_angles[i] = get_closest_angle(target_joint_angles[i], curr_joint_angle) # ensure we take the shortest path
        #     joint_errors[i] = target_joint_angles[i] - curr_joint_angle
        #     # simple P control
        #     vel = joint_pd_gains[i]['kp'] * joint_errors[i]
        #     set_angle = curr_joint_angle + np.clip(vel, -joint_max_vel[i], joint_max_vel[i]) * 0.01 # assuming 100Hz update rate
        #     # set_angle = get_closest_angle(set_angle, curr_joint_angle) # ensure we take the shortest path
        #     set_joint(f"joint{joint_order[i]}", set_angle)

        set_joint("joint5", thetas[0] * joint_inversions[0] + joint_offsets[0])
        set_joint("joint0", thetas[1] * joint_inversions[1] + joint_offsets[1])
        set_joint("joint1", thetas[2] * joint_inversions[2] + joint_offsets[2])
        set_joint("joint2", thetas[3] * joint_inversions[3] + joint_offsets[3])
        set_joint("joint3", thetas[4] * joint_inversions[4] + joint_offsets[4])
        set_joint("joint4", thetas[5] * joint_inversions[5] + joint_offsets[5])

        # mujoco forward kinematics update

        # draw target position
        target_pos = np.array(state['target_pos'], dtype=np.float64)
        target_rot = np.array(state['target_rot'], dtype=np.float64).reshape(3, 3)
        euler_angles = R.Rotation.from_matrix(target_rot).as_euler('xyz', degrees=False)
        # euler_angles[-1] += joint_offsets[0]
        # target_rot = R.Rotation.from_euler('xyz', euler_angles).as_matrix()

        # target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
        # data.xpos[target_id] = target_pos

        # target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
        # data.xpos[target_id] = np.array([0.3, 0.0, 0.5])  # 30cm X, 0 Y, 50cm Z

        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "target_joint")
        qpos_addr = model.jnt_qposadr[joint_id]
        data.qpos[qpos_addr:qpos_addr+3] = [target_pos[0], target_pos[1], target_pos[2]]  # note the negation for x and y to match coordinate systems
        # data.qpos[qpos_addr+3:qpos_addr+7] = [1,0,0,0] # identity quaternion
        data.qpos[qpos_addr+3:qpos_addr+7] = rot_mat_to_quat(target_rot)



        mujoco.mj_forward(model, data)

        viewer.sync()

        # draw_target(viewer, target_pos, target_rot)
        # draw_test_sphere(viewer, target_pos)
        # print(len(viewer.user_scn.geoms))

        time.sleep(0.01)

client_thread.stop()