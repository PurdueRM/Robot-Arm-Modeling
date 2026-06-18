// arm_test.c
// Build: gcc arm_test.c -o arm_test
// Linux/macOS only (uses termios for raw keyboard input)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "simplified_arm.h"

// ── config ───────────────────────────────────────────────────────────────────

#define UDP_PORT       5005
#define BROADCAST_IP   "255.255.255.255"
#define STEP_LINEAR    0.005f   // metres per keypress
#define STEP_ANGULAR   5.0f     // degrees per keypress
#define STEP_AXIS      0.005f   // metres per keypress

// ── terminal raw mode ────────────────────────────────────────────────────────

static struct termios g_orig_termios;

static void restore_terminal(void) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
}

static void enable_raw_mode(void) {
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    atexit(restore_terminal);

    struct termios raw = g_orig_termios;
    raw.c_lflag    &= ~(ICANON | ECHO);   // no line buffering, no echo
    raw.c_cc[VMIN]  = 0;                  // non-blocking reads
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

// ── UDP broadcast ────────────────────────────────────────────────────────────

static int g_sock = -1;
static struct sockaddr_in g_dest;

static void udp_init(void) {
    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) { perror("socket"); exit(1); }

    int broadcast = 1;
    setsockopt(g_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    memset(&g_dest, 0, sizeof(g_dest));
    g_dest.sin_family      = AF_INET;
    g_dest.sin_port        = htons(UDP_PORT);
    g_dest.sin_addr.s_addr = inet_addr(BROADCAST_IP);
}

static void udp_send(const ArmState* state) {
    // Pack 6 floats as network-byte-order 32-bit IEEE 754.
    // On any modern system floats are already IEEE 754; we just fix endianness.
    uint32_t buf[6];
    const float angles[6] = {
        state->theta1, state->theta2, state->theta3,
        state->theta4, state->theta5, state->theta6
    };
    for (int i = 0; i < 6; i++) {
        uint32_t tmp;
        memcpy(&tmp, &angles[i], sizeof tmp);
        buf[i] = htonl(tmp);
    }
    sendto(g_sock, buf, sizeof buf, 0,
           (struct sockaddr*)&g_dest, sizeof g_dest);
}

// ── display ──────────────────────────────────────────────────────────────────

static void print_state(const ArmState* s) {
    // Move cursor to top-left, no clear (avoids flicker)
    printf("\033[H");
    printf("┌─────────────────────────────────────┐\n");
    printf("│         ARM STATE (degrees)          │\n");
    printf("├──────────────┬──────────────────────┤\n");
    printf("│ θ1 shoulder  │ %8.2f°             │\n", s->theta1);
    printf("│ θ2 elbow     │ %8.2f°             │\n", s->theta2);
    printf("│ θ3 roll      │ %8.2f°             │\n", s->theta3);
    printf("│ θ4 wrist 1   │ %8.2f°             │\n", s->theta4);
    printf("│ θ5 wrist 2   │ %8.2f°             │\n", s->theta5);
    printf("│ θ6 gripper   │ %8.2f°             │\n", s->theta6);
    printf("├──────────────┴──────────────────────┤\n");
    printf("│ Controls:                            │\n");
    printf("│  W/S   translate X  (fwd/back)       │\n");
    printf("│  A/D   translate Z  (up/down)        │\n");
    printf("│  ↑/↓   pitch wrist  (θ4)             │\n");
    printf("│  ←/→   roll  wrist  (θ3)             │\n");
    printf("│  J/K   move along axis               │\n");
    printf("│  Q     quit                          │\n");
    printf("└─────────────────────────────────────┘\n");
    fflush(stdout);
}

// ── main ─────────────────────────────────────────────────────────────────────

int main(void) {
    ArmState state = {
        .theta1 = 45.0f,
        .theta2 = 90.0f,
        .theta3 =  0.0f,
        .theta4 = 45.0f,
        .theta5 = 90.0f,
        .theta6 =  0.0f,
    };

    enable_raw_mode();
    udp_init();

    // Clear screen once, then use cursor-home for updates
    printf("\033[2J");

    print_state(&state);
    udp_send(&state);

    while (1) {
        unsigned char c = 0;
        int n = (int)read(STDIN_FILENO, &c, 1);

        if (n <= 0) {
            usleep(10000);   // 10 ms sleep when no input
            continue;
        }

        int changed = 1;

        if (c == 'q' || c == 'Q') break;

        // WASD ── shoulder/elbow plane
        else if (c == 'w' || c == 'W') translate_x(&state,  STEP_LINEAR);
        else if (c == 's' || c == 'S') translate_x(&state, -STEP_LINEAR);
        else if (c == 'a' || c == 'A') translate_z(&state,  STEP_LINEAR);
        else if (c == 'd' || c == 'D') translate_z(&state, -STEP_LINEAR);

        // J/K ── axis movement
        else if (c == 'j' || c == 'J') move_along_axis(&state,  STEP_AXIS);
        else if (c == 'k' || c == 'K') move_along_axis(&state, -STEP_AXIS);

        // Escape sequences for arrow keys: ESC [ A/B/C/D
        else if (c == 0x1b) {
            unsigned char seq[2] = {0, 0};
            // Both bytes should arrive immediately after ESC
            if (read(STDIN_FILENO, &seq[0], 1) == 1 && seq[0] == '[') {
                if (read(STDIN_FILENO, &seq[1], 1) == 1) {
                    switch (seq[1]) {
                        case 'A': pitch_wrist(&state,  STEP_ANGULAR); break; // up
                        case 'B': pitch_wrist(&state, -STEP_ANGULAR); break; // down
                        case 'C': roll_wrist (&state,  STEP_ANGULAR); break; // right
                        case 'D': roll_wrist (&state, -STEP_ANGULAR); break; // left
                        default:  changed = 0; break;
                    }
                }
            } else {
                changed = 0;
            }
        }

        else {
            changed = 0;
        }

        if (changed) {
            print_state(&state);
            udp_send(&state);
        }
    }

    close(g_sock);
    return 0;
}