import pygame
import pygame.gfxdraw
import sdl2
import sdl2.ext
from ctypes import c_float
import math
import ctypes
import numpy as np
import os

import json
from network_utils import UDP_Server

PORT = 65432

os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

# SDL2 controller/sensor init 
sdl2.SDL_Init(sdl2.SDL_INIT_GAMECONTROLLER | sdl2.SDL_INIT_SENSOR)
controller = sdl2.SDL_GameControllerOpen(0)
sdl2.SDL_GameControllerSetSensorEnabled(controller, sdl2.SDL_SENSOR_GYRO, True)
sdl2.SDL_GameControllerSetSensorEnabled(controller, sdl2.SDL_SENSOR_ACCEL, True)
gyro_data  = (c_float * 3)()
accel_data = (c_float * 3)()

pygame.init()
WINDOW_W, WINDOW_H = 700, 550
screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
pygame.display.set_caption("Controller Sensors")
clock  = pygame.time.Clock()
font_s = pygame.font.SysFont("monospace", 11)
font_m = pygame.font.SysFont("monospace", 13, bold=True)

SPHERE_CX, SPHERE_CY, SPHERE_R = 110, 110, 85
BAR_X, BAR_W, BAR_H = 230, 300, 13
BAR_GAP = 7

BG       = (18, 18, 18)
GRID_COL = (50, 50, 50)
TRACK_COL= (38, 38, 38)

AXIS_DEFS = [
    ([1,0,0], (220, 60,  60),  (90, 25, 25),  "X"),
    ([0,1,0], (70,  190,  50), (28, 80, 18),  "Y"),
    ([0,0,1], (55,  130, 230), (18, 55, 110), "Z"),
]
GYRO_COLORS  = [(220, 80,  80),  (80, 200, 60),  (60, 140, 230)]
ACCEL_COLORS = [(230, 120, 60),  (100, 210, 80), (80, 160, 240)]

# quaternions stuff
def mul_quat(a, b):
    return [
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    ]

def quat_from_axis_angle(ax, ay, az, angle):
    s = math.sin(angle / 2)
    return [math.cos(angle / 2), ax*s, ay*s, az*s]

def rotate_vec(q, v):
    qv = [0, v[0], v[1], v[2]]
    qc = [q[0], -q[1], -q[2], -q[3]]
    r  = mul_quat(mul_quat(q, qv), qc)
    return [r[1], r[2], r[3]]

def project(v):
    x = SPHERE_CX + int(v[0] * SPHERE_R * 0.92)
    y = SPHERE_CY - int(v[1] * SPHERE_R * 0.92)
    return x, y

def quaternion_to_euler(x, y, z, w):
    """
    Convert quat (x, y, z, w) to Euler angles (roll, pitch, yaw)
    roll = rotation around x-axis
    pitch = rotation around y-axis
    yaw = rotation around z-axis

    Returns angles in radians.
    """

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# ueler stuff

def euler_to_matrix(roll, pitch, yaw):
    """i use convention: roll (x), pitch (y), yaw (z)"""
    cx, cy, cz = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sx, sy, sz = math.sin(roll), math.sin(pitch), math.sin(yaw)

    R = np.array([
        [cy*cz, cz*sx*sy - cx*sz, sx*sz + cx*cz*sy],
        [cy*sz, cx*cz + sx*sy*sz, cx*sy*sz - cz*sx],
        [-sy,   cy*sx,             cx*cy]
    ])
    return R


def matrix_to_euler(R):
    """XYZ convention"""
    sy = -R[2, 0]
    
    if abs(sy) < 1 - 1e-6:
        pitch = math.asin(sy)
        roll  = math.atan2(R[2, 1], R[2, 2])
        yaw   = math.atan2(R[1, 0], R[0, 0])
    else:
        # gimbal lock
        pitch = math.asin(sy)
        roll  = math.atan2(-R[1, 2], R[1, 1])
        yaw   = 0

    return roll, pitch, yaw


def cylindrical_to_world_rotation(x, y):
    """rot matrix from (r, θ, z) frame to world frame"""
    theta = math.atan2(y, x)

    R = np.array([
        [ math.cos(theta), -math.sin(theta), 0],
        [ math.sin(theta),  math.cos(theta), 0],
        [ 0,                0,               1]
    ])
    return R


def local_cylindrical_to_world_euler(x, y, z, roll_r, pitch_theta, yaw_z):
    """
    convert euler angles defined in (r, θ, z) frame
    into world frame (x, y, z) euler angles.
    """

    # cyl to world
    R_cyl_to_world = cylindrical_to_world_rotation(x, y)

    # local cyl rot mat
    R_local = euler_to_matrix(roll_r, pitch_theta, yaw_z)

    # combine
    R_world = R_cyl_to_world @ R_local
    return matrix_to_euler(R_world)

# draw stuff

def draw_sphere_grid(surf, q):
    # i asked chat for this lol
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

def draw_axis(surf, q, axis_vec, color, neg_color, label):
    # i asked chat for this lol
    neg = [-axis_vec[0], -axis_vec[1], -axis_vec[2]]
    rv  = rotate_vec(q, axis_vec)
    rn  = rotate_vec(q, neg)

    # Negative half (dim)
    ex, ey = project(rn)
    pygame.draw.line(surf, neg_color, (SPHERE_CX, SPHERE_CY), (ex, ey), 1)

    # Positive half
    px, py = project(rv)
    pygame.gfxdraw.line(surf, SPHERE_CX, SPHERE_CY, px, py, color)

    # Arrowhead
    dx, dy = px - SPHERE_CX, py - SPHERE_CY
    length  = math.hypot(dx, dy)
    if length > 1:
        nx, ny = dx/length, dy/length
        ox, oy = -ny*5, nx*5
        tip    = (px, py)
        left   = (int(px - nx*12 + ox), int(py - ny*12 + oy))
        right  = (int(px - nx*12 - ox), int(py - ny*12 - oy))
        pygame.draw.polygon(surf, color, [tip, left, right])

    # Axis label just beyond tip
    lx = SPHERE_CX + (rv[0] * SPHERE_R * 1.15)
    ly = SPHERE_CY - (rv[1] * SPHERE_R * 1.15)
    txt = font_s.render(label, True, color)
    surf.blit(txt, txt.get_rect(center=(int(lx), int(ly))))

def draw_bar(surf, y, value, max_val, color, label):
    # i asked chat for this lol
    center_x = BAR_X + BAR_W // 2
    pct      = min(1.0, abs(value) / max_val)
    fill_w   = int(pct * (BAR_W // 2))

    # Track
    pygame.draw.rect(surf, TRACK_COL, (BAR_X, y, BAR_W, BAR_H), border_radius=3)

    # Fill
    if value >= 0:
        pygame.draw.rect(surf, color, (center_x, y, fill_w, BAR_H), border_radius=3)
    else:
        pygame.draw.rect(surf, color, (center_x - fill_w, y, fill_w, BAR_H), border_radius=3)

    # Center divider
    pygame.draw.line(surf, (90, 90, 90), (center_x, y), (center_x, y + BAR_H))

    # Label + value
    lbl = font_s.render(label, True, (150, 150, 150))
    surf.blit(lbl, (BAR_X - 28, y + 1))
    val_txt = font_s.render(f"{value:+.2f}", True, color)
    surf.blit(val_txt, (BAR_X + BAR_W + 6, y + 1))

DT = 1 / 60.0
running = True

data_state = {
    'pitch': 0.0,
    'yaw': 0.0,
    'roll': 0.0,
    'x': 0.0,
    'y': 0.0,
    'z': 0.0
}
server = UDP_Server(host='127.0.0.1',port=PORT, broadcast_enabled=True)
server.start()

joystick = pygame.joystick.Joystick(0)
joystick.init()

quat = [1.0, 0.0, 0.0, 0.0]
position = [0.0, 0.0, 0.0]
velocity = [0.0, 0.0, 0.0]
reference_quat = quat[:]
reference_accel_state = None
gyro_drift = None

rolling_avg_x = np.zeros(10)
rolling_avg_y = np.zeros(10)
rolling_avg_z = np.zeros(10)
rolling_index = 0

imu_update_disabled = False
imu_toggle_button_pressed = False

relative_orientation_mode = False
relative_mode_button_pressed = False

def safe_btn(i):
    try:    return joystick.get_button(i)
    except: return False
def safe_axis(i, deadband=0.1):
    try:    
        val = joystick.get_axis(i)
        if abs(val) < deadband:
            return 0.0
        return val
    except: return 0.0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # if a button pressed -> reset orientation and position
        # if event.type == sdl2.SDL_CONTROLLERBUTTONDOWN:
        #     print("Button pressed, resetting orientation and position")
        #     if event.button == sdl2.SDL_CONTROLLER_BUTTON_A:
        #         print("Resetting orientation and position")
        #         quat = [1.0, 0.0, 0.0, 0.0]
        #         position = [0.0, 0.0, 0.0]
        #         velocity = [0.0, 0.0, 0.0]    



    # read sensor
    sdl2.SDL_PumpEvents()
    sdl2.SDL_GameControllerGetSensorData(controller, sdl2.SDL_SENSOR_GYRO,  gyro_data,  3)
    sdl2.SDL_GameControllerGetSensorData(controller, sdl2.SDL_SENSOR_ACCEL, accel_data, 3)

    gx, gy, gz = gyro_data[0], gyro_data[1], gyro_data[2]
    ax, ay, az = accel_data[0], accel_data[1], accel_data[2]

    if safe_btn(1):
        # print("Button pressed, resetting orientation and position")
        quat = [1.0, 0.0, 0.0, 0.0]
        # position = [0.0, 0.0, 0.0]
        position = [0.3, 0.3, 0.0]
        velocity = [0.0, 0.0, 0.0]
        reference_accel_state = [ax, ay, az]
        gyro_drift = [gx, gy, gz]

    if reference_accel_state is None:
        reference_accel_state = [ax, ay, az]
    
    if gyro_drift is None:
        gyro_drift = [gx, gy, gz]
    
    # offset by gravity or other accel
    # ax = ax - reference_accel_state[0]
    # ay = ay - reference_accel_state[1]
    # az = az - reference_accel_state[2]

    # Integrate gyro → quaternion
    gx = gx - gyro_drift[0]
    gy = gy - gyro_drift[1]
    gz = gz - gyro_drift[2]

    if safe_btn(2):
        if not imu_toggle_button_pressed:
            imu_update_disabled = not imu_update_disabled
            imu_toggle_button_pressed = True
            print(f"IMU updates {'disabled' if imu_update_disabled else 'enabled'}")
    else:
        imu_toggle_button_pressed = False

    if safe_btn(3):
        if not relative_mode_button_pressed:
            relative_orientation_mode = not relative_orientation_mode
            relative_mode_button_pressed = True
            if relative_orientation_mode:
                print("Relative orientation mode enabled")
            else:
                print("Relative orientation mode disabled")
    else:
        relative_mode_button_pressed = False

    if safe_btn(5):
        quat = [1.0, 0.0, 0.0, 0.0]

    if not safe_btn(0): # if b held, ignore updates
        angle = math.sqrt(gx*gx + gy*gy + gz*gz) * DT
        if angle > 1e-6:
            mag = angle / DT
            dq  = quat_from_axis_angle(gx/mag, gy/mag, gz/mag, angle)
            quat = mul_quat(quat, dq)
            n    = math.sqrt(sum(x*x for x in quat))
            quat = [x/n for x in quat]

        if not imu_update_disabled:
            # get acceleration in world frame
            ax_w, ay_w, az_w = rotate_vec(quat, [ax, ay, az])
            # # remove gravity (assuming controller is mostly upright)
            # az_w -= 9.81
            ax_w = ax_w - reference_accel_state[0]
            ay_w = ay_w - reference_accel_state[1]
            az_w = az_w - reference_accel_state[2]

            rolling_avg_x[rolling_index] = ax_w
            rolling_avg_y[rolling_index] = ay_w
            rolling_avg_z[rolling_index] = az_w
            rolling_index = (rolling_index + 1) % len(rolling_avg_x)

            mean_ax = np.mean(rolling_avg_x)
            mean_ay = np.mean(rolling_avg_y)
            mean_az = np.mean(rolling_avg_z)
            var_ax = np.var(rolling_avg_x)
            var_ay = np.var(rolling_avg_y)
            var_az = np.var(rolling_avg_z)

            var_lims = [0.1, 0.1, 0.1]
            if var_ax < var_lims[0]:
                ax_w = 0.0
                velocity[0] = 0.0
            if var_ay < var_lims[1]:
                ay_w = 0.0
                velocity[1] = 0.0
            if var_az < var_lims[2]:
                az_w = 0.0
                velocity[2] = 0.0

            # Integrate accel → velocity → position
            # I zero velocity as the user is unlikely to want to have drift -> onnly change when moved
            # if abs(ax_w) < 0.4:
            #     velocity[0] = 0.0
            # if abs(ay_w) < 0.4:
            #     velocity[1] = 0.0
            # if abs(az_w) < 0.9:
            #     velocity[2] = 0.0

            velocity[0] += ax_w * DT
            velocity[1] += ay_w * DT
            velocity[2] += az_w * DT
            position[0] += velocity[0] * DT
            position[1] += velocity[1] * DT
            position[2] += velocity[2] * DT
        else:
            ax_w, ay_w, az_w = 0.0, 0.0, 0.0
    else:
        ax_w, ay_w, az_w = 0.0, 0.0, 0.0

    screen.fill(BG)

    pygame.gfxdraw.aacircle(screen, SPHERE_CX, SPHERE_CY, SPHERE_R, GRID_COL)

    draw_sphere_grid(screen, quat)

    # sort axes by depth
    sorted_axes = sorted(AXIS_DEFS, key=lambda a: rotate_vec(quat, a[0])[2])
    for (axis_vec, color, neg_color, label) in sorted_axes:
        draw_axis(screen, quat, axis_vec, color, neg_color, label)

    # headers
    TEXT_PADDING = 24
    screen.blit(font_m.render("GYRO  (rad/s)", True, (100,100,100)), (BAR_X - 28, 12))
    screen.blit(font_m.render("ACCEL (m/s²)",  True, (100,100,100)), (BAR_X - 28, 118))
    screen.blit(font_m.render("ACCEL OFFSET (m/s²)",  True, (100,100,100)), (BAR_X - 28, 118 + TEXT_PADDING + BAR_H*3 + BAR_GAP*3))

    gyro_vals  = [gx, gy, gz]
    accel_vals = [ax, ay, az]

    for i in range(3):
        y = 30 + i * (BAR_H + BAR_GAP)
        draw_bar(screen, y, gyro_vals[i],  10.0, GYRO_COLORS[i],  ["GX","GY","GZ"][i])

    for i in range(3):
        y = 136 + i * (BAR_H + BAR_GAP)
        draw_bar(screen, y, accel_vals[i], 20.0, ACCEL_COLORS[i], ["AX","AY","AZ"][i])

    accel_offset = [ax_w, ay_w, az_w]
    for i in range(3):
        y = 136 + BAR_H*3 + BAR_GAP*3 + i * (BAR_H + BAR_GAP) + TEXT_PADDING
        draw_bar(screen, y, accel_offset[i], 20.0, (80,80,80), ["OFFX","OFFY","OFFZ"][i])

    # draw rectangle for x-y field and point for x-y position
    FIELD_SIZE = 200
    FIELD_X = BAR_X
    FIELD_Y = WINDOW_H - FIELD_SIZE - 40
    pygame.draw.rect(screen, (50,50,50), (FIELD_X, FIELD_Y, FIELD_SIZE, FIELD_SIZE), 1)
    pos_x = FIELD_X + FIELD_SIZE // 2 - int(-position[2] * FIELD_SIZE // 2) # plot y pos on x axis
    pos_y = FIELD_Y + FIELD_SIZE // 2 - int(position[0] * FIELD_SIZE // 2) # plot x pos on y axis
    pygame.gfxdraw.filled_circle(screen, pos_x, pos_y, 5, (200, 80, 80))
    y_axis_text = font_m.render("<- +   x   - ->", True, (150, 150, 150))
    x_axis_text = font_m.render("<- +   y   - ->", True, (150, 150, 150))
    rotated_y_axis_text = pygame.transform.rotate(y_axis_text, 90)
    screen.blit(rotated_y_axis_text, (FIELD_X + FIELD_SIZE + 10, FIELD_Y + FIELD_SIZE // 2 - rotated_y_axis_text.get_height() // 2))
    screen.blit(x_axis_text, (FIELD_X + FIELD_SIZE // 2 - x_axis_text.get_width() // 2, FIELD_Y + FIELD_SIZE + 5))

    # z bar thing
    BAR_Z_X = FIELD_X + FIELD_SIZE + 40
    BAR_Z_Y = FIELD_Y
    BAR_Z_W = 20
    BAR_Z_H = FIELD_SIZE
    pygame.draw.rect(screen, (50,50,50), (BAR_Z_X, BAR_Z_Y, BAR_Z_W, BAR_Z_H), 1)
    pos_z = BAR_Z_Y + BAR_Z_H // 2 - int(position[1] * BAR_Z_H // 2)
    pygame.draw.rect(screen, (200, 80, 80), (BAR_Z_X + 2, pos_z - 5, BAR_Z_W - 4, 10))
    z_axis_text = font_m.render("<- +   z   - ->", True, (150, 150, 150))
    rotated_z_axis_text = pygame.transform.rotate(z_axis_text, 90)
    screen.blit(rotated_z_axis_text, (BAR_Z_X + BAR_Z_W + 10, BAR_Z_Y + BAR_Z_H // 2 - rotated_z_axis_text.get_height() // 2))

    # draw indicators for modes
    mode_text = "Relative Orientation Mode: ON" if relative_orientation_mode else "Relative Orientation Mode: OFF"
    mode_color = (80, 200, 80) if relative_orientation_mode else (200, 80, 80)
    mode_txt = font_m.render(mode_text, True, mode_color)
    screen.blit(mode_txt, (30, FIELD_Y - 40))

    mode_text = "IMU Updates: OFF" if imu_update_disabled else "IMU Updates: ON"
    mode_color = (200, 80, 80) if imu_update_disabled else (80, 200, 80)
    mode_txt = font_m.render(mode_text, True, mode_color)
    screen.blit(mode_txt, (30, FIELD_Y - 70))

    # check if joysticks used to offset position
    lx = safe_axis(0); ly = safe_axis(1)
    rx = safe_axis(2); ry = safe_axis(3)

    rate = 0.5 # m/s
    position[0] += -ly * rate * DT
    position[1] += -ry * rate * DT
    position[2] += lx * rate * DT


    position = np.clip(position, -0.8, 0.8).tolist()

    euler_roll, euler_pitch, euler_yaw = quaternion_to_euler(*quat)

    rot_angle_text = font_s.render(f"Roll: {math.degrees(euler_roll):.1f}°, Pitch: {math.degrees(euler_pitch):.1f}°, Yaw: {math.degrees(euler_yaw):.1f}°", True, (150, 150, 150))
    screen.blit(rot_angle_text, (30, 30))

    if relative_orientation_mode:
        # euler_roll, euler_pitch, euler_yaw = local_cylindrical_to_world_euler(position[0], -position[2], position[1], euler_roll, euler_pitch, euler_yaw)
        euler_roll, euler_pitch, euler_yaw = local_cylindrical_to_world_euler(position[0], -position[2], position[1], euler_roll, -euler_yaw, 0.0)
    else:
        euler_pitch = -euler_yaw
        euler_yaw = euler_pitch

    # print(f"Euler angles (rad): roll={euler_roll:.2f}, pitch={euler_pitch:.2f}, yaw={euler_yaw:.2f}")
    data_state['pitch'] = np.rad2deg(euler_pitch) # -np.rad2deg(euler_yaw) # y robot = -z controller
    data_state['yaw']   = np.rad2deg(euler_yaw) # np.rad2deg(euler_pitch) # z robot = y controller
    data_state['roll']  = np.rad2deg(euler_roll) # np.rad2deg(euler_roll)
    data_state['x'] = position[0]
    data_state['y'] = -position[2] # -z axis is y axis of robot 
    data_state['z'] = position[1] # y axis is z axis of robot
    server.broadcast(json.dumps(data_state))

    pygame.display.flip()
    clock.tick(60)

server.stop()

sdl2.SDL_GameControllerClose(controller)
sdl2.SDL_Quit()
pygame.quit()