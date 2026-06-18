import pygame
import pygame.gfxdraw
import math
import threading
import asyncio
import numpy as np
import os
import json
from network_utils import UDP_Server
from bleak import BleakClient
from scipy.spatial.transform import Rotation as R

# esp32 ble config
ADDRESS = "30:76:F5:B9:B7:C6"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

PORT = 65432

# layout
SPHERE_CX, SPHERE_CY, SPHERE_R = 110, 110, 85
BAR_X, BAR_W, BAR_H = 230, 300, 13
BAR_GAP = 7

BG = (18,  18,  18)
GRID_COL = (50,  50,  50)
TRACK_COL = (38,  38,  38)

RED_L = (220, 60, 60)
RED_D = (90, 25, 25)
GREEN_L = (70, 190, 50)
GREEN_D = (28, 80, 18)
BLUE_L = (55, 130, 230)
BLUE_D = (18, 55, 110)

AXIS_DEFS = [
    ([1, 0, 0], RED_L, RED_D, "X"),
    ([0, 1, 0], GREEN_L, GREEN_D, "Y"),
    ([0, 0, 1], BLUE_L, BLUE_D, "Z"),
]
GYRO_COLORS  = [RED_L, GREEN_L, BLUE_L]
ACCEL_COLORS = [RED_D, GREEN_D, BLUE_D]


# quaternion math
# q: [x, y, z, w]
# (vector part first, scalar part last)
def mul_quat(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b

    return [
        aw*bx + ax*bw + ay*bz - az*by, # x
        aw*by - ax*bz + ay*bw + az*bx, # y
        aw*bz + ax*by - ay*bx + az*bw, # z
        aw*bw - ax*bx - ay*by - az*bz # w
    ]


def quat_from_axis_angle(ax, ay, az, angle):
    s = math.sin(angle / 2.0)
    c = math.cos(angle / 2.0)
    return [ax*s, ay*s, az*s, c]


def quat_conjugate(q):
    x, y, z, w = q
    return [-x, -y, -z, w]


def rotate_vec(q, v):
    # Promote vector to pure quaternion
    qv = [v[0], v[1], v[2], 0.0]

    # Rotate: q * v * q*
    r = mul_quat(mul_quat(q, qv), quat_conjugate(q))
    return r[:3]


def project(v):
    x = SPHERE_CX + int(v[0] * SPHERE_R * 0.92)
    y = SPHERE_CY - int(v[1] * SPHERE_R * 0.92)
    return x, y


def quaternion_to_euler(x, y, z, w):
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# euler/rot mat math
def euler_to_matrix(roll, pitch, yaw):
    cx, cy, cz = math.cos(roll),  math.cos(pitch), math.cos(yaw)
    sx, sy, sz = math.sin(roll),  math.sin(pitch), math.sin(yaw)
    return np.array([
        [cy*cz, cz*sx*sy - cx*sz, sx*sz + cx*cz*sy],
        [cy*sz, cx*cz + sx*sy*sz, cx*sy*sz - cz*sx],
        [-sy,   cy*sx,             cx*cy            ],
    ])

def matrix_to_euler(R):
    sy = -R[2, 0]
    if abs(sy) < 1 - 1e-6:
        pitch = math.asin(sy)
        roll  = math.atan2(R[2, 1], R[2, 2])
        yaw   = math.atan2(R[1, 0], R[0, 0])
    else:
        pitch = math.asin(sy)
        roll  = math.atan2(-R[1, 2], R[1, 1])
        yaw   = 0.0
    return roll, pitch, yaw

def cylindrical_to_world_rotation(x, y):
    theta = math.atan2(y, x)
    return np.array([
        [ math.cos(theta), -math.sin(theta), 0],
        [ math.sin(theta),  math.cos(theta), 0],
        [ 0,                0,               1],
    ])

def local_cylindrical_to_world_euler(x, y, z, roll_r, pitch_theta, yaw_z):
    R_cyl   = cylindrical_to_world_rotation(x, y)
    R_local = euler_to_matrix(roll_r, pitch_theta, yaw_z)
    return matrix_to_euler(R_cyl @ R_local)


# parse notification
def parse_ble_line(line: str):
    """
    notif: ax,ay,az,roll_deg,pitch_deg,yaw_deg
    angle in deg, return as rad
    """
    try:
        parts = line.strip().split(',')
        if len(parts) != 6:
            return None
        # timestamp = float(parts[0])
        ax, ay, az = map(float, parts[0:3])
        roll, pitch, yaw = map(np.deg2rad, map(float, parts[3:6]))
        return {
            # 'timestamp': timestamp,
            'accel':     (ax * 9.80665, ay * 9.80665, az * 9.80665),
            'euler':     (roll, pitch, yaw),
        }
    except ValueError:
        return None


# Draw stuff (asked chat for this)
def draw_sphere_grid(surf, q):
    for lat_deg in range(-60, 61, 30):
        ry = math.sin(math.radians(lat_deg))
        rr = math.cos(math.radians(lat_deg))
        prev = None
        for lon_deg in range(0, 364, 5):
            lx = math.cos(math.radians(lon_deg)) * rr
            lz = math.sin(math.radians(lon_deg)) * rr
            v  = rotate_vec(q, [lx, ry, lz])
            if v[2] < 0:
                prev = None
                continue
            px, py = project(v)
            if prev:
                pygame.gfxdraw.line(surf, prev[0], prev[1], px, py, GRID_COL)
            prev = (px, py)

    for lon_deg in range(0, 180, 30):
        prev = None
        for lat_deg in range(-90, 91, 5):
            ry = math.sin(math.radians(lat_deg))
            rr = math.cos(math.radians(lat_deg))
            lx = math.cos(math.radians(lon_deg)) * rr
            lz = math.sin(math.radians(lon_deg)) * rr
            v  = rotate_vec(q, [lx, ry, lz])
            if v[2] < 0:
                prev = None
                continue
            px, py = project(v)
            if prev:
                pygame.gfxdraw.line(surf, prev[0], prev[1], px, py, GRID_COL)
            prev = (px, py)

def draw_axis(surf, q, axis_vec, color, neg_color, label, font_s):
    neg = [-axis_vec[0], -axis_vec[1], -axis_vec[2]]
    rv  = rotate_vec(q, axis_vec)
    rn  = rotate_vec(q, neg)
    ex, ey = project(rn)
    pygame.draw.line(surf, neg_color, (SPHERE_CX, SPHERE_CY), (ex, ey), 1)
    px, py = project(rv)
    pygame.gfxdraw.line(surf, SPHERE_CX, SPHERE_CY, px, py, color)
    dx, dy = px - SPHERE_CX, py - SPHERE_CY
    length = math.hypot(dx, dy)
    if length > 1:
        nx, ny = dx/length, dy/length
        ox, oy = -ny*5, nx*5
        tip   = (px, py)
        left  = (int(px - nx*12 + ox), int(py - ny*12 + oy))
        right = (int(px - nx*12 - ox), int(py - ny*12 - oy))
        pygame.draw.polygon(surf, color, [tip, left, right])
    lx = SPHERE_CX + rv[0] * SPHERE_R * 1.15
    ly = SPHERE_CY - rv[1] * SPHERE_R * 1.15
    txt = font_s.render(label, True, color)
    surf.blit(txt, txt.get_rect(center=(int(lx), int(ly))))

def draw_bar(surf, y, value, max_val, color, label, font_s):
    center_x = BAR_X + BAR_W // 2
    pct      = min(1.0, abs(value) / max_val)
    fill_w   = int(pct * (BAR_W // 2))
    pygame.draw.rect(surf, TRACK_COL, (BAR_X, y, BAR_W, BAR_H), border_radius=3)
    if value >= 0:
        pygame.draw.rect(surf, color, (center_x,          y, fill_w, BAR_H), border_radius=3)
    else:
        pygame.draw.rect(surf, color, (center_x - fill_w, y, fill_w, BAR_H), border_radius=3)
    pygame.draw.line(surf, (90, 90, 90), (center_x, y), (center_x, y + BAR_H))
    lbl = font_s.render(label, True, (150, 150, 150))
    surf.blit(lbl, (BAR_X - 28, y + 1))
    val_txt = font_s.render(f"{value:+.2f}", True, color)
    surf.blit(val_txt, (BAR_X + BAR_W + 6, y + 1))


# BLE receiver class 
class BLEReceiver:
    def __init__(self, address: str, char_uuid: str):
        self.address = address
        self.char_uuid = char_uuid
        self._lock = threading.Lock()
        self._latest = None # most-recent parsed packet
        self._loop = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def get_latest(self):
        with self._lock:
            pkt = self._latest
            self._latest = None
        return pkt

    def _notification_handler(self, sender, data):
        try:
            line = data.decode('utf-8')
            parsed = parse_ble_line(line)
            if parsed:
                with self._lock:
                    self._latest = parsed
        except Exception as e:
            print(f"[BLE] decode error: {e}")

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._connect_loop())

    async def _connect_loop(self):
        while True:
            print(f"[BLE] Connecting to {self.address} …")
            try:
                async with BleakClient(self.address) as client:
                    if client.is_connected:
                        print("[BLE] Connected.")
                        await client.start_notify(self.char_uuid, self._notification_handler)
                        while client.is_connected:
                            await asyncio.sleep(0.01)
                        print("[BLE] Disconnected, retrying …")
                    else:
                        print("[BLE] Failed to connect, retrying …")
            except Exception as e:
                print(f"[BLE] Error: {e}  — retrying in 2 s")
                await asyncio.sleep(2)


def main():
    ble = BLEReceiver(ADDRESS, CHARACTERISTIC_UUID)
    ble.start()

    pygame.init()
    WINDOW_W, WINDOW_H = 700, 550
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("IMU Visualiser (BLE)")
    clock  = pygame.time.Clock()
    font_s = pygame.font.SysFont("monospace", 11)
    font_m = pygame.font.SysFont("monospace", 13, bold=True)

    DT = 1 / 60.0

    data_state = {'pitch': 0.0, 'yaw': 0.0, 'roll': 0.0,
                  'x': 0.0, 'y': 0.0, 'z': 0.0}

    server = UDP_Server(host='127.0.0.1', port=PORT, broadcast_enabled=True)
    server.start()

    quat = [1.0, 0.0, 0.0, 0.0]
    position = [0.0, 0.0, 0.0]
    velocity = [0.0, 0.0, 0.0]
    reference_quat = None
    reference_accel = None

    rolling_avg_x  = np.zeros(10)
    rolling_avg_y  = np.zeros(10)
    rolling_avg_z  = np.zeros(10)
    rolling_index  = 0

    imu_update_disabled       = False
    relative_orientation_mode = False

    # last sensor values for the HUD
    ax = ay = az = 0.0
    ax_w = ay_w = az_w = 0.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    print("Resetting orientation and position")
                    reference_quat  = None
                    quat = [0.0, 0.0, 0.0, 1.0]
                    position = [0.3, 0.3, 0.0]
                    velocity = [0.0, 0.0, 0.0]

                if event.key == pygame.K_t:
                    imu_update_disabled = not imu_update_disabled
                    print(f"IMU updates {'disabled' if imu_update_disabled else 'enabled'}")

                if event.key == pygame.K_y:
                    relative_orientation_mode = not relative_orientation_mode
                    print(f"Relative orientation mode {'enabled' if relative_orientation_mode else 'disabled'}")

                # x-y pos move
                if event.key == pygame.K_UP:    position[0] += 0.1
                if event.key == pygame.K_DOWN:  position[0] -= 0.1
                if event.key == pygame.K_LEFT:  position[2] += 0.1
                if event.key == pygame.K_RIGHT: position[2] -= 0.1

                # z axis move & rotate about z
                if event.key == pygame.K_w: position[1] += 0.1
                if event.key == pygame.K_s: position[1] -= 0.1
                if event.key == pygame.K_a:
                    # dq   = quat_from_axis_angle(0, 1, 0, math.radians(10))
                    # quat = mul_quat(quat, dq)
                    curr_x = position[0]
                    curr_y = position[1]
                    delta_angle = math.radians(10)
                    cos_a = math.cos(delta_angle)
                    sin_a = math.sin(delta_angle)
                    position[0] = curr_x * cos_a - curr_y * sin_a
                    position[1] = curr_x * sin_a + curr_y * cos_a
                if event.key == pygame.K_d:
                    # dq   = quat_from_axis_angle(0, 1, 0, math.radians(-10))
                    # quat = mul_quat(quat, dq)
                    curr_x = position[0]
                    curr_y = position[1]
                    delta_angle = math.radians(-10)
                    cos_a = math.cos(delta_angle)
                    sin_a = math.sin(delta_angle)
                    position[0] = curr_x * cos_a - curr_y * sin_a
                    position[1] = curr_x * sin_a + curr_y * cos_a

        # get ble notif from esp32
        pkt = ble.get_latest()
        if pkt:
            ax, ay, az = pkt['accel']
            roll, pitch, yaw  = pkt['euler']

            # euler to quat  (roll=x, pitch=y, yaw=z)
            q = R.from_euler('xyz', [roll, pitch, yaw]).as_quat() # returns in xyzw order
            curr_imu_quat = [q[0], q[1], q[2], q[3]] # convert to xyzw list

            if reference_quat is None:
                reference_quat = curr_imu_quat[:]

            # delta from reference
            quat = mul_quat(curr_imu_quat,
                            quat_conjugate(reference_quat))

            if reference_accel is None:
                reference_accel = [ax, ay, az]

            if not imu_update_disabled:
                # subtract gravity (note the imu acc is backwards)
                g_vec = rotate_vec(quat_conjugate(curr_imu_quat), [0, 0, 9.80665])
                ax -= g_vec[0]
                ay -= g_vec[1]
                az -= g_vec[2]

                ax_w, ay_w, az_w = rotate_vec(quat, [ax, ay, az])
                # ax_w -= reference_accel[0]
                # ay_w -= reference_accel[1]
                # az_w -= reference_accel[2]

                rolling_avg_x[rolling_index] = ax_w
                rolling_avg_y[rolling_index] = ay_w
                rolling_avg_z[rolling_index] = az_w
                rolling_index = (rolling_index + 1) % len(rolling_avg_x)

                var_lims = [0.01, 0.01, 0.01]
                if np.var(rolling_avg_x) < var_lims[0]:
                    ax_w = 0.0;  velocity[0] = 0.0
                if np.var(rolling_avg_y) < var_lims[1]:
                    ay_w = 0.0;  velocity[1] = 0.0
                if np.var(rolling_avg_z) < var_lims[2]:
                    az_w = 0.0;  velocity[2] = 0.0

                velocity[0] += ax_w * DT;  position[0] += velocity[0] * DT
                velocity[1] += ay_w * DT;  position[1] += velocity[1] * DT
                velocity[2] += az_w * DT;  position[2] += velocity[2] * DT
            else:
                ax_w = ay_w = az_w = 0.0

        # draw
        screen.fill(BG)
        pygame.gfxdraw.aacircle(screen, SPHERE_CX, SPHERE_CY, SPHERE_R, GRID_COL)
        draw_sphere_grid(screen, quat)

        for axis_def in sorted(AXIS_DEFS, key=lambda a: rotate_vec(quat, a[0])[2]):
            draw_axis(screen, quat, *axis_def, font_s)

        TEXT_PADDING = 24
        screen.blit(font_m.render("GYRO  (rad/s)",         True, (100,100,100)), (BAR_X - 28, 12))
        screen.blit(font_m.render("ACCEL (m/s²)",          True, (100,100,100)), (BAR_X - 28, 118))
        screen.blit(font_m.render("ACCEL OFFSET (m/s²)",   True, (100,100,100)),
                    (BAR_X - 28, 118 + TEXT_PADDING + BAR_H*3 + BAR_GAP*3))

        # no gyro acc from esp 32 but was from joystick
        for i, lbl in enumerate(["GX", "GY", "GZ"]):
            draw_bar(screen, 30  + i*(BAR_H+BAR_GAP), 0.0,            10.0, GYRO_COLORS[i],  lbl, font_s)
        for i, (val, lbl) in enumerate(zip([ax, ay, az], ["AX","AY","AZ"])):
            draw_bar(screen, 136 + i*(BAR_H+BAR_GAP), val,            20.0, ACCEL_COLORS[i], lbl, font_s)
        for i, (val, lbl) in enumerate(zip([ax_w, ay_w, az_w], ["OFFX","OFFY","OFFZ"])):
            draw_bar(screen,
                     136 + BAR_H*3 + BAR_GAP*3 + i*(BAR_H+BAR_GAP) + TEXT_PADDING,
                     val, 20.0, (80,80,80), lbl, font_s)

        # position field
        FIELD_SIZE = 200
        FIELD_X    = BAR_X
        FIELD_Y    = WINDOW_H - FIELD_SIZE - 40
        pygame.draw.rect(screen, (50,50,50), (FIELD_X, FIELD_Y, FIELD_SIZE, FIELD_SIZE), 1)
        pos_x = FIELD_X + FIELD_SIZE//2 - int( position[1] * FIELD_SIZE//2)
        pos_y = FIELD_Y + FIELD_SIZE//2 - int( position[0] * FIELD_SIZE//2)
        pygame.gfxdraw.filled_circle(screen, pos_x, pos_y, 5, (200, 80, 80))

        y_ax = pygame.transform.rotate(font_m.render("<- +   x   - ->", True, (150,150,150)), 90)
        screen.blit(y_ax, (FIELD_X + FIELD_SIZE + 10,
                            FIELD_Y + FIELD_SIZE//2 - y_ax.get_height()//2))
        x_ax = font_m.render("<- +   y   - ->", True, (150,150,150))
        screen.blit(x_ax, (FIELD_X + FIELD_SIZE//2 - x_ax.get_width()//2,
                            FIELD_Y + FIELD_SIZE + 5))

        # z bar
        BAR_Z_X = FIELD_X + FIELD_SIZE + 40
        pygame.draw.rect(screen, (50,50,50), (BAR_Z_X, FIELD_Y, 20, FIELD_SIZE), 1)
        pos_z = FIELD_Y + FIELD_SIZE//2 - int(position[2] * FIELD_SIZE//2)
        pygame.draw.rect(screen, (200, 80, 80), (BAR_Z_X+2, pos_z-5, 16, 10))
        z_ax = pygame.transform.rotate(font_m.render("<- +   z   - ->", True, (150,150,150)), 90)
        screen.blit(z_ax, (BAR_Z_X + 30,
                            FIELD_Y + FIELD_SIZE//2 - z_ax.get_height()//2))

        # mode indicators
        for text, flag, y_off in [
            ("IMU Updates: OFF" if imu_update_disabled else "IMU Updates: ON",
             not imu_update_disabled, FIELD_Y - 70),
            ("Relative Orientation Mode: ON" if relative_orientation_mode else "Relative Orientation Mode: OFF",
             relative_orientation_mode, FIELD_Y - 40),
        ]:
            color = (80, 200, 80) if flag else (200, 80, 80)
            screen.blit(font_m.render(text, True, color), (30, y_off))

        # euler readout
        euler_roll, euler_pitch, euler_yaw = quaternion_to_euler(*quat)
        screen.blit(
            font_s.render(
                f"Roll: {math.degrees(euler_roll):.1f}°, "
                f"Pitch: {math.degrees(euler_pitch):.1f}°, "
                f"Yaw: {math.degrees(euler_yaw):.1f}°",
                True, (150,150,150)),
            (30, 30))

        # clamp position
        position = np.clip(position, -0.8, 0.8).tolist()

        # broadcast
        if relative_orientation_mode:
            euler_roll, euler_pitch, euler_yaw = local_cylindrical_to_world_euler(
                position[0], position[1], position[2],
                euler_roll, euler_pitch, 0.0)
        # else:
        #     euler_pitch = -euler_yaw
        #     euler_yaw   = euler_pitch

        # convert from rollx, pitchy, yawz to pitchy, yawz, rollx for the robot arm
        rot_mat = R.from_euler('xyz', [euler_roll, euler_pitch, euler_yaw]).as_matrix()
        euler_pitch, euler_yaw, euler_roll = R.from_matrix(rot_mat).as_euler('yxz')

        data_state['pitch'] = np.rad2deg(euler_pitch)
        data_state['yaw']   = np.rad2deg(euler_yaw)
        data_state['roll']  = np.rad2deg(euler_roll)
        data_state['x'] = position[0]
        data_state['y'] = position[1]
        data_state['z'] = position[2]
        server.broadcast(json.dumps(data_state))

        pygame.display.flip()
        clock.tick(60)

    server.stop()
    pygame.quit()


if __name__ == "__main__":
    main()