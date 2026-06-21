import numpy as np
import pygame
from pygame.locals import *
from network_utils import UDP_Server, TCP_Server
import socket
import json

PORT = 65432

SCREEN_W = 1000
SCREEN_H = 700
PANEL_W = 380
INPUT_W = SCREEN_W - PANEL_W
MARGIN = 16


BG = (18, 20, 28)
PANEL_BG = (24, 26, 36)
BORDER = (55, 60, 85)
ACCENT = (80, 200, 140) # teal
ACCENT2 = (100, 140, 255) # blue
INACTIVE = (60, 65, 90)
TEXT = (210, 215, 235)
TEXT_DIM = (110, 115, 145)
WIDGET_BG = (28, 30, 42)
STICK_RING = (70, 75, 100)
TRIGGER_BG = (35, 38, 55)

# helpers
def lerp(a, b, t):
    return a + (b - a) * t

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def draw_rounded_rect(surf, color, rect, radius=8, width=0):
    pygame.draw.rect(surf, color, rect, width, border_radius=radius)

def map_range(v, in_lo, in_hi, out_lo, out_hi):
    t = clamp((v - in_lo) / (in_hi - in_lo), 0, 1)
    return lerp(out_lo, out_hi, t)

# widget sizing
def compute_rects():
    usable_w = INPUT_W - 2 * MARGIN
    
    slider_h = 70
    pad_h    = slider_h * 3
    gap      = MARGIN
    total_h  = pad_h + gap + pad_h + gap + slider_h + gap + slider_h
    y0 = (SCREEN_H - total_h) // 2

    rects = {}
    y = y0
    rects['pitch_yaw'] = pygame.Rect(MARGIN, y, usable_w, pad_h);   y += pad_h + gap
    rects['x_y']       = pygame.Rect(MARGIN, y, usable_w, pad_h);   y += pad_h + gap
    rects['roll']      = pygame.Rect(MARGIN, y, usable_w, slider_h); y += slider_h + gap
    rects['z']         = pygame.Rect(MARGIN, y, usable_w, slider_h)
    return rects

def draw_2d_widget(surf, font_sm, widget):
    r   = widget['rect']
    val = widget['value']
    bx0, bx1 = widget['bounds'][0]
    by0, by1 = widget['bounds'][1]

    draw_rounded_rect(surf, WIDGET_BG, r, 10)
    draw_rounded_rect(surf, BORDER,    r, 10, 2)

    # border
    cx = r.x + r.w // 2
    cy = r.y + r.h // 2
    pygame.draw.line(surf, BORDER, (r.x+8, cy), (r.right-8, cy), 1)
    pygame.draw.line(surf, BORDER, (cx, r.y+8), (cx, r.bottom-8), 1)

    # dot position 
    px = int(map_range(val[0], bx0, bx1, r.x+8, r.right-8))
    py = int(map_range(val[1], by0, by1, r.y+8, r.bottom-8))

    # ring + dot
    glow_surf = pygame.Surface((30, 30), pygame.SRCALPHA)
    pygame.draw.circle(glow_surf, (*ACCENT, 50), (15, 15), 14)
    surf.blit(glow_surf, (px-15, py-15))
    pygame.draw.circle(surf, ACCENT, (px, py), 7)
    pygame.draw.circle(surf, TEXT,   (px, py), 7, 2)

    # label
    vx_str = f"{val[0]:.1f}"
    vy_str = f"{val[1]:.1f}"
    lbl = font_sm.render(f"{widget['name']}  x={vx_str}  y={vy_str}", True, TEXT_DIM)
    surf.blit(lbl, (r.x+8, r.y+5))

    # axis labels
    for label, pos in [
        (f"{bx0}", (r.x+4, cy-10)),
        (f"{bx1}", (r.right-28, cy-10)),
        (f"{by0}", (cx+4, r.y+4)),
        (f"{by1}", (cx+4, r.bottom-18)),
    ]:
        t = font_sm.render(label, True, BORDER)
        surf.blit(t, pos)


def draw_slider_widget(surf, font_sm, widget):
    r   = widget['rect']
    val = widget['value']
    lo, hi = widget['bounds']
    rel = clamp((val - lo) / (hi - lo), 0, 1)

    draw_rounded_rect(surf, WIDGET_BG, r, 8)
    draw_rounded_rect(surf, BORDER,    r, 8, 2)

    # track
    track_y  = r.y + r.h // 2
    track_x0 = r.x + 12
    track_x1 = r.right - 12
    pygame.draw.line(surf, INACTIVE, (track_x0, track_y), (track_x1, track_y), 3)

    # filled portion
    fill_x = int(lerp(track_x0, track_x1, rel))
    if fill_x > track_x0:
        pygame.draw.line(surf, ACCENT2, (track_x0, track_y), (fill_x, track_y), 3)

    # thumb
    pygame.draw.circle(surf, ACCENT2, (fill_x, track_y), 9)
    pygame.draw.circle(surf, TEXT,    (fill_x, track_y), 9, 2)

    lbl = font_sm.render(f"{widget['name']}  {val:.2f}", True, TEXT_DIM)
    surf.blit(lbl, (r.x+8, r.y+6))

    lo_t = font_sm.render(str(lo), True, BORDER)
    hi_t = font_sm.render(str(hi), True, BORDER)
    surf.blit(lo_t, (track_x0, r.bottom-18))
    surf.blit(hi_t, (track_x1-hi_t.get_width(), r.bottom-18))


# joystick
def draw_joystick_panel(surf, font, font_sm, font_tiny, joy, enabled, panel_rect):
    draw_rounded_rect(surf, PANEL_BG, panel_rect, 12)
    draw_rounded_rect(surf, BORDER,   panel_rect, 12, 2)

    px, py = panel_rect.x + 16, panel_rect.y + 16
    pw     = panel_rect.width - 32

    # title
    title = font.render("CONTROLLER", True, TEXT)
    surf.blit(title, (px + pw//2 - title.get_width()//2, py))
    py += title.get_height() + 6

    if not enabled:
        msg = font_sm.render("Enable to connect", True, TEXT_DIM)
        surf.blit(msg, (px + pw//2 - msg.get_width()//2, py + 40))
        return

    if joy is None:
        msg = font_sm.render("No controller detected", True, TEXT_DIM)
        surf.blit(msg, (px + pw//2 - msg.get_width()//2, py + 40))
        return

    def safe_axis(i):
        try:    return joy.get_axis(i)
        except: return 0.0
    def safe_btn(i):
        try:    return joy.get_button(i)
        except: return False

    # axis
    lx = safe_axis(0); ly = safe_axis(1)
    rx = safe_axis(2); ry = safe_axis(3)
    # lt = (safe_axis(4) + 1) / 2
    # rt = (safe_axis(5) + 1) / 2
    lt = safe_btn(7); rt = safe_btn(8)   # pro controller doesn't have analog triggers :(  

    # triggers
    trig_w  = 60; trig_h = 70
    trig_y  = py + 10
    lt_rect = pygame.Rect(px, trig_y, trig_w, trig_h)
    rt_rect = pygame.Rect(px + pw - trig_w, trig_y, trig_w, trig_h)

    for rect, val, label, col in [
        (lt_rect, lt, "LT", ACCENT2),
        (rt_rect, rt, "RT", ACCENT),
    ]:
        draw_rounded_rect(surf, TRIGGER_BG, rect, 6)
        fill_h = int(rect.height * val)
        fill_r = pygame.Rect(rect.x, rect.bottom - fill_h, rect.width, fill_h)
        draw_rounded_rect(surf, col, fill_r, 4)
        draw_rounded_rect(surf, BORDER, rect, 6, 2)
        lbl = font_tiny.render(label, True, TEXT_DIM)
        surf.blit(lbl, (rect.x + rect.w//2 - lbl.get_width()//2, rect.y - 16))
        pct = font_tiny.render(f"{int(val*100)}%", True, TEXT)
        surf.blit(pct, (rect.x + rect.w//2 - pct.get_width()//2, rect.bottom + 2))

    py += trig_h + 32

    # shoulder
    lb = safe_btn(5); rb = safe_btn(6)
    sh_w = 70; sh_h = 18
    for btn, lbl_str, bx, col in [
        (lb, "LB", px, ACCENT2),
        (rb, "RB", px + pw - sh_w, ACCENT),
    ]:
        col_fill = col if btn else INACTIVE
        draw_rounded_rect(surf, col_fill, pygame.Rect(bx, py, sh_w, sh_h), 5)
        t = font_tiny.render(lbl_str, True, TEXT)
        surf.blit(t, (bx + sh_w//2 - t.get_width()//2, py + 2))
    py += sh_h + 30

    # +/- buttons

    min_btn = safe_btn(9); plus_btn = safe_btn(10)
    for btn, lbl_str, bx, col in [
        (min_btn, "MIN", px, ACCENT2),
        (plus_btn, "PLUS", px + pw - sh_w, ACCENT),
    ]:
        col_fill = col if btn else INACTIVE
        draw_rounded_rect(surf, col_fill, pygame.Rect(bx, py, sh_w, sh_h), 5)
        t = font_tiny.render(lbl_str, True, TEXT)
        surf.blit(t, (bx + sh_w//2 - t.get_width()//2, py + 2))
    py += sh_h + 30

    # stick
    stick_r   = 44
    center_y  = py + stick_r + 6
    lcx = px + pw // 4
    rcx = px + 3 * pw // 4

    for cx, cy, ax, ay, label, col in [
        (lcx, center_y, lx, ly, "L", ACCENT2),
        (rcx, center_y, rx, ry, "R", ACCENT),
    ]:
        # outer ring
        pygame.draw.circle(surf, STICK_RING, (cx, cy), stick_r, 2)
        # crosshair
        pygame.draw.line(surf, STICK_RING, (cx-stick_r+4, cy), (cx+stick_r-4, cy), 1)
        pygame.draw.line(surf, STICK_RING, (cx, cy-stick_r+4), (cx, cy+stick_r-4), 1)
        # dot
        dx = cx + int(ax * (stick_r - 10))
        dy = cy + int(ay * (stick_r - 10))
        # glow
        gs = pygame.Surface((24, 24), pygame.SRCALPHA)
        pygame.draw.circle(gs, (*col, 60), (12, 12), 12)
        surf.blit(gs, (dx-12, dy-12))
        pygame.draw.circle(surf, col, (dx, dy), 8)
        pygame.draw.circle(surf, TEXT, (dx, dy), 8, 2)

        # axis values
        ax_t = font_tiny.render(f"x:{ax:+.2f} y:{ay:+.2f}", True, TEXT_DIM)
        surf.blit(ax_t, (cx - ax_t.get_width()//2, cy + stick_r + 4))

        lbl_s = font_sm.render(label, True, TEXT_DIM)
        surf.blit(lbl_s, (cx - lbl_s.get_width()//2, cy - stick_r - 18))

    py += stick_r * 2 + 80

    # ABXY buttons
    face_labels = ['B','A','X','Y']
    face_colors = [(80,200,100),(220,80,80),(80,130,255),(220,200,60)]
    face_offsets = [(0,1),(1,0),(0,-1),(-1,0)]   # ABXY diamond
    fc_cx = px + pw // 2
    fc_cy = py + 30
    fr    = 14
    gap_f = 26

    for i in range(min(4, joy.get_numbuttons() if joy else 0)):
        pressed = safe_btn(i)
        ox, oy  = face_offsets[i]
        bx_c = fc_cx + ox * gap_f
        by_c = fc_cy + oy * gap_f
        col  = face_colors[i] if pressed else INACTIVE
        pygame.draw.circle(surf, col, (bx_c, by_c), fr)
        pygame.draw.circle(surf, TEXT if pressed else BORDER, (bx_c, by_c), fr, 2)
        t = font_tiny.render(face_labels[i], True, TEXT)
        surf.blit(t, (bx_c - t.get_width()//2, by_c - t.get_height()//2))

    py += 100

    # dpad
    if joy.get_numhats() > 0:
        hat = joy.get_hat(0)
        dpx = px + pw // 2
        dpy = py + 20
        dp_arm = 14; dp_w = 12

        for hx, hy in [(1,0),(-1,0),(0,1),(0,-1)]:
            pressed = (hat[0] == hx and hat[1] == hy)
            arm_rect = pygame.Rect(
                dpx + hx * dp_arm - (dp_w if hx == 0 else dp_arm)//2 + (dp_w//2 if hx != 0 else 0),
                dpy - hy * dp_arm - (dp_w if hy == 0 else dp_arm)//2 + (dp_w//2 if hy != 0 else 0),
                dp_arm if hx == 0 else dp_arm,
                dp_arm if hy == 0 else dp_arm,
            )

            if   (hx,hy) == (1,0):  arm_rect = pygame.Rect(dpx+dp_w//2,     dpy-dp_w//2, dp_arm, dp_w)
            elif (hx,hy) == (-1,0): arm_rect = pygame.Rect(dpx-dp_arm-dp_w//2, dpy-dp_w//2, dp_arm, dp_w)
            elif (hx,hy) == (0,1):  arm_rect = pygame.Rect(dpx-dp_w//2,     dpy-dp_arm-dp_w//2, dp_w, dp_arm)
            elif (hx,hy) == (0,-1): arm_rect = pygame.Rect(dpx-dp_w//2,     dpy+dp_w//2, dp_w, dp_arm)
            
            draw_rounded_rect(surf, ACCENT if pressed else INACTIVE, arm_rect, 3)

        for hx, hy in [(1,1),(-1,1),(-1,-1),(1,-1)]:
            pressed = (hat[0] == hx and hat[1] == hy)
            pygame.draw.circle(surf, ACCENT if pressed else INACTIVE, (dpx + hx * dp_arm, dpy - hy * dp_arm), 6)


        t = font_tiny.render("D-PAD", True, TEXT_DIM)
        surf.blit(t, (dpx - t.get_width()//2, dpy + 30))

    # legend 
    legend_y = panel_rect.bottom - 90
    for line in ["L-Stick → Pitch/Yaw", "R-Stick → X/Y",
                 "LT/RT → Z", "LB/RB → Roll"]:
        t = font_tiny.render(line, True, TEXT_DIM)
        surf.blit(t, (px + pw//2 - t.get_width()//2, legend_y))
        legend_y += 16


def draw_toggle(surf, font_sm, rect, enabled):
    color = ACCENT if enabled else INACTIVE
    draw_rounded_rect(surf, color, rect, rect.height // 2)
    knob_x = rect.right - rect.height//2 - 2 if enabled else rect.x + rect.height//2 + 2
    pygame.draw.circle(surf, TEXT, (knob_x, rect.centery), rect.height//2 - 3)
    lbl = font_sm.render("Joystick" if not enabled else "Joystick ON", True, TEXT)
    surf.blit(lbl, (rect.x - lbl.get_width() - 10, rect.centery - lbl.get_height()//2))


def main():
    pygame.init()
    pygame.joystick.init()

    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Controller UI")

    font       = pygame.font.SysFont("monospace", 16, bold=True)
    font_sm    = pygame.font.SysFont("monospace", 14)
    font_tiny  = pygame.font.SysFont("monospace", 12)

    rects = compute_rects()

    widgets = {
        'pitch_yaw': {
            'name': 'Pitch / Yaw', 'rect': rects['pitch_yaw'],
            'type': '2dinput', 'active': False,
            'value': [0.0, 0.0],
            'bounds': [(-90, 90), (-180, 180)],
        },
        'x_y': {
            'name': 'X / Y', 'rect': rects['x_y'],
            'type': '2dinput', 'active': False,
            'value': [0.0, 0.0],
            'bounds': [(-0.8, 0.8), (-0.8, 0.8)],
        },
        'roll': {
            'name': 'Roll', 'rect': rects['roll'],
            'type': 'slider', 'active': False,
            'value': 0.0,
            'bounds': (-180, 180),
        },
        'z': {
            'name': 'Z', 'rect': rects['z'],
            'type': 'slider', 'active': False,
            'value': 0.5,
            'bounds': (0, 1.0),
        },
    }

    joystick_enabled = False
    joystick         = None
    active_widget    = None
    mouse_down       = False

    toggle_rect = pygame.Rect(INPUT_W - 120, SCREEN_H - 36, 50, 22)
    panel_rect  = pygame.Rect(INPUT_W, 12, PANEL_W - 12, SCREEN_H - 24)

    clock = pygame.time.Clock()
    done  = False

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

    while not done:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == QUIT:
                done = True

            elif event.type == MOUSEBUTTONDOWN and event.button == 1:
                mouse_down = True

                # toggle
                if toggle_rect.collidepoint(event.pos):
                    joystick_enabled = not joystick_enabled
                    if joystick_enabled and pygame.joystick.get_count() > 0:
                        joystick = pygame.joystick.Joystick(0)
                        joystick.init()
                    else:
                        joystick = None

                # widget pick
                active_widget = None
                for key, w in widgets.items():
                    if w['rect'].collidepoint(event.pos):
                        active_widget = key

            elif event.type == MOUSEBUTTONUP and event.button == 1:
                mouse_down    = False
                active_widget = None

            elif event.type == MOUSEMOTION and mouse_down and active_widget:
                w   = widgets[active_widget]
                r   = w['rect']
                mx, my = event.pos

                if w['type'] == 'slider':
                    rel = clamp((mx - r.x) / r.width, 0, 1)
                    lo, hi = w['bounds']
                    w['value'] = lerp(lo, hi, rel)

                elif w['type'] == '2dinput':
                    bx0, bx1 = w['bounds'][0]
                    by0, by1 = w['bounds'][1]
                    relx = clamp((mx - r.x) / r.width,  0, 1)
                    rely = clamp((my - r.y) / r.height, 0, 1)
                    w['value'][0] = lerp(bx0, bx1, relx)
                    w['value'][1] = lerp(by0, by1, rely)

        # hoystc + widgets
        if joystick_enabled and joystick:
            def safe_axis(i):
                try:    return joystick.get_axis(i)
                except: return 0.0
            def safe_btn(i):
                try:    return joystick.get_button(i)
                except: return False

            lx = safe_axis(0); ly = safe_axis(1)
            rx = safe_axis(2); ry = safe_axis(3)
            # lt = (safe_axis(4) + 1) / 2
            # rt = (safe_axis(5) + 1) / 2
            lt = safe_btn(7); rt = safe_btn(8) 
            lb = safe_btn(5)
            rb = safe_btn(6)
            min_btn = safe_btn(9)
            plus_btn = safe_btn(10)

            DEAD = 0.08
            # RATE = 120   # units/sec

            # left stick -> pitch yaw
            if abs(lx) > DEAD or abs(ly) > DEAD:
                rate_pitch = 120
                rate_yaw = 120
                ly = -ly # invert y
                bx0, bx1 = widgets['pitch_yaw']['bounds'][0]
                by0, by1 = widgets['pitch_yaw']['bounds'][1]
                widgets['pitch_yaw']['value'][0] = clamp(
                    widgets['pitch_yaw']['value'][0] + ly * rate_pitch * dt, bx0, bx1)
                widgets['pitch_yaw']['value'][1] = clamp(
                    widgets['pitch_yaw']['value'][1] + lx * rate_yaw * dt, by0, by1)

            # rigt stick -> x/y
            if abs(rx) > DEAD or abs(ry) > DEAD:
                rate_x = 0.4
                rate_y = 0.4
                ry = -ry # invert y
                bx0, bx1 = widgets['x_y']['bounds'][0]
                by0, by1 = widgets['x_y']['bounds'][1]
                widgets['x_y']['value'][0] = clamp(
                    widgets['x_y']['value'][0] + rx * rate_x * dt, bx0, bx1)
                widgets['x_y']['value'][1] = clamp(
                    widgets['x_y']['value'][1] + ry * rate_y * dt, by0, by1)

            # trigger -> z
            z_lo, z_hi = widgets['z']['bounds']
            z_val = (rb - lb) * 0.1 * dt
            # z_val = lerp(z_lo, z_hi, (rt - lt + 1) / 2)
            widgets['z']['value'] += z_val

            # shoulder -> roll
            rate_roll = 120
            r_lo, r_hi = widgets['roll']['bounds']
            roll_speed = (plus_btn - min_btn) * rate_roll * dt
            widgets['roll']['value'] = clamp(
                widgets['roll']['value'] + roll_speed, r_lo, r_hi)

        # drwa
        screen.fill(BG)

        # send
        data_state['pitch'] = widgets['pitch_yaw']['value'][0]
        data_state['yaw'] = widgets['pitch_yaw']['value'][1]
        data_state['x'] = widgets['x_y']['value'][0]
        data_state['y'] = widgets['x_y']['value'][1]
        data_state['z'] = widgets['z']['value']
        data_state['roll'] = widgets['roll']['value']
        server.broadcast(json.dumps(data_state))

        pygame.draw.line(screen, BORDER, (INPUT_W, 0), (INPUT_W, SCREEN_H), 1)

        # widgets
        for key, w in widgets.items():
            if w['type'] == '2dinput':
                draw_2d_widget(screen, font_sm, w)
            else:
                draw_slider_widget(screen, font_sm, w)

        # toggle
        draw_toggle(screen, font_sm, toggle_rect, joystick_enabled)

        # joystick
        draw_joystick_panel(screen, font, font_sm, font_tiny,
                            joystick, joystick_enabled, panel_rect)

        pygame.display.flip()

    server.stop()

    pygame.quit()


if __name__ == "__main__":
    main()