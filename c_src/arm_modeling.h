#include <stdio.h>
#include <math.h>

#include "user_math.h"

/*
l1: length of first arm
l2: length of second arm
h1: height from base of shoulder motor
h2: offset of second arm (it is L shaped)
h3: offset of spherical wrist center
h4: height of motor (dist from center of spherical wrist to end effector)
*/
#define L1 0.4
#define L2 0.3
#define H1 0.1
#define H2 0.1
#define H3 0.05
#define H4 0.05

#define DAMPING 0.01
#define EPSILON 0.000001

typedef struct joint_t {
    DH_Params dh_params;
    float theta;
    float velocity;
    float x_pos;
    float y_pos;
    float z_pos;
} joint_t;

typedef struct Arm_State_t {
    joint_t joints[6];
    joint_t sim_joints[6];

    Mat *jacobian;
    Mat *pseudo_inv_jacobian;
    Mat *transforms[7];
    Mat *sim_transforms[7];

    Mat *desired_orientation_mat;
    Mat *current_orientation_mat;
    Vec *orientation_error;

    Vec *wrist_pos;
    Vec *wrist_pos_desired;

    Vec *desired_pos;

    Vec *theta13_compute;
    Vec *theta46_compute;

    // the buffers exist to avoid malloc/free in the control loop
    Mat *buffer1_4x4;
    Mat *buffer2_4x4;

    Mat *buffer1_3x3;
    Mat *buffer2_3x3;

    Vec *buffer1_3x1;
    Vec *buffer2_3x1;
    
} Arm_State_t;

Arm_State_t g_arm_state;