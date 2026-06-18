#include <stdio.h>
#include <math.h>

#include "user_math.h"
#include "arm_modeling.h"


// [a, alpha, d, theta]
Arm_State_t g_arm_state = { .joints[0].dh_params = {0.0f, PI/2, H1, 0.0f},
                            .joints[1].dh_params = {L1, 0.0f, 0.0f, PI/2},
                            .joints[2].dh_params = {H2, PI/2, 0, PI/2},
                            .joints[3].dh_params = {0.0f, -1 * PI/2, L2 + H3, 0.0f},
                            .joints[4].dh_params = {0.0f, -1 * PI/2, 0.0f, 0.0f},
                            .joints[5].dh_params = {0.0f, 0.0f, H4, 0.0f},

                            .sim_joints[0].dh_params = {0.0f, PI/2, H1, 0.0f},
                            .sim_joints[1].dh_params = {L1, 0.0f, 0.0f, PI/2},
                            .sim_joints[2].dh_params = {H2, PI/2, 0, PI/2},
                            .sim_joints[3].dh_params = {0.0f, -1 * PI/2, L2 + H3, 0.0f},
                            .sim_joints[4].dh_params = {0.0f, -1 * PI/2, 0.0f, 0.0f},
                            .sim_joints[5].dh_params = {0.0f, 0.0f, H4, 0.0f}
};

Mat* joint_transform(joint_t joint) {
    return dh_transform(joint.dh_params);
}

Mat* joint_transform_buffer(joint_t joint, Mat* buffer) {
    return dh_transform_buffer(joint.dh_params, buffer);
}

void forward_kinematics(Arm_State_t *arm_state) {
    dh_update_arm();

    for (int i = 0; i < 6; i++) {
        assert(arm_state->transforms[0] != NULL);
    }
    
    for (int i = 0; i < 6; i++) {
        joint_transform_buffer(arm_state->joints[i], arm_state->buffer1_4x4);
        mat_mult_buffer(arm_state->transforms[i], arm_state->buffer1_4x4, arm_state->transforms[i+1]);
        g_arm_state.joints[i].x_pos = MAT_IDX(arm_state->transforms[i+1], 0,3);
        g_arm_state.joints[i].y_pos = MAT_IDX(arm_state->transforms[i+1], 1,3);
        g_arm_state.joints[i].z_pos = MAT_IDX(arm_state->transforms[i+1], 2,3);
    }
}

void dh_update_arm() {
    g_arm_state.joints[0].dh_params.theta = g_arm_state.joints[0].theta;
    g_arm_state.joints[1].dh_params.theta = g_arm_state.joints[1].theta;
    g_arm_state.joints[2].dh_params.theta = g_arm_state.joints[2].theta;
    g_arm_state.joints[3].dh_params.theta = g_arm_state.joints[3].theta;
    g_arm_state.joints[4].dh_params.theta = g_arm_state.joints[4].theta;
    g_arm_state.joints[5].dh_params.theta = g_arm_state.joints[5].theta;
}

void compute_orientation_error(Arm_State_t *arm_state) {
    Vec *rotvec = arm_state->orientation_error;

    mat_transpose_buffer(arm_state->current_orientation_mat, arm_state->buffer1_3x3);
    mat_mult_buffer(arm_state->desired_orientation_mat, arm_state->buffer1_3x3, arm_state->buffer2_3x3);

    // compute rotation angle theta
    float trace = mat_trace(arm_state->buffer2_3x3);
    float theta = acos(clamp((trace - 1) / 2, -1.0f, 1.0f));
    float s = sin(theta);

    if (fabs(theta) < 1e-6) {
        VEC_IDX(rotvec, 0) = 0.0f;
        VEC_IDX(rotvec, 1) = 0.0f;
        VEC_IDX(rotvec, 2) = 0.0f;
        return;
    }

    // compute rotation vector
    VEC_IDX(rotvec, 0) = (MAT_IDX(arm_state->buffer2_3x3, 2, 1) - MAT_IDX(arm_state->buffer2_3x3, 1, 2)) / (2.0f * s);
    VEC_IDX(rotvec, 1) = (MAT_IDX(arm_state->buffer2_3x3, 0, 2) - MAT_IDX(arm_state->buffer2_3x3, 2, 0)) / (2.0f * s);
    VEC_IDX(rotvec, 2) = (MAT_IDX(arm_state->buffer2_3x3, 1, 0) - MAT_IDX(arm_state->buffer2_3x3, 0, 1)) / (2.0f * s);

    // scale by theta
    mat_scalar_mult_buffer(rotvec, theta, rotvec);
}

void get_wrist_center(Arm_State_t *arm_state) {
    // get z axis of target orientation
    VEC_IDX(arm_state->buffer1_3x1, 0) = MAT_IDX(arm_state->desired_orientation_mat, 0, 2);
    VEC_IDX(arm_state->buffer1_3x1, 1) = MAT_IDX(arm_state->desired_orientation_mat, 1, 2);
    VEC_IDX(arm_state->buffer1_3x1, 2) = MAT_IDX(arm_state->desired_orientation_mat, 2, 2);

    // offset pos by H3 to get wrist pos
    mat_scalar_mult_buffer(arm_state->buffer1_3x1, H3, arm_state->buffer1_3x1);
    mat_add_buffer(arm_state->desired_pos, arm_state->buffer1_3x1, arm_state->wrist_pos_desired);
}

void calculate_position_jacobian(Arm_State_t *arm_state) {
    /*
    J11 = (h₂⋅cos(θ₂ - θ₃) + h₃⋅sin(θ₂ - θ₃) + l₁⋅sin(θ₂) + l₂⋅sin(θ₂ - θ₃))⋅sin(θ₁)
    J12 = (h₂⋅sin(θ₂ - θ₃) - h₃⋅cos(θ₂ - θ₃) - l₁⋅cos(θ₂) - l₂⋅cos(θ₂ - θ₃))⋅cos(θ₁)
    J13 = (-h₂⋅sin(θ₂ - θ₃) + h₃⋅cos(θ₂ - θ₃) + l₂⋅cos(θ₂ - θ₃))⋅cos(θ₁)
    J21 = -(h₂⋅cos(θ₂ - θ₃) + h₃⋅sin(θ₂ - θ₃) + l₁⋅sin(θ₂) + l₂⋅sin(θ₂ - θ₃))⋅cos(θ₁)
    J22 = (h₂⋅sin(θ₂ - θ₃) - h₃⋅cos(θ₂ - θ₃) - l₁⋅cos(θ₂) - l₂⋅cos(θ₂ - θ₃))⋅sin(θ₁)
    J23 = (-h₂⋅sin(θ₂ - θ₃) + h₃⋅cos(θ₂ - θ₃) + l₂⋅cos(θ₂ - θ₃))⋅sin(θ₁)
    J31 = 0
    J32 = -h₂⋅cos(θ₂ - θ₃) - h₃⋅sin(θ₂ - θ₃) - l₁⋅sin(θ₂) - l₂⋅sin(θ₂ - θ₃)
    J33 = h₂⋅cos(θ₂ - θ₃) + h₃⋅sin(θ₂ - θ₃) + l₂⋅sin(θ₂ - θ₃)
    */

    float theta1 = arm_state->joints[0].theta;
    float theta2 = arm_state->joints[2].theta;
    float theta3 = arm_state->joints[3].theta;

    float sin_theta1 = sin(theta1);
    float cos_theta1 = cos(theta1);
    float sin_theta2 = sin(theta2);
    float cos_theta2 = cos(theta2);
    float sin_theta23 = sin(theta2 - theta3);
    float cos_theta23 = cos(theta2 - theta3);

    MAT_IDX(arm_state->jacobian, 0, 0) = (H2 * cos_theta23 + H3 * sin_theta23 + L1 * sin_theta2 + L2 * sin_theta23) * sin_theta1;
    MAT_IDX(arm_state->jacobian, 0, 1) = (H2 * sin_theta23 - H3 * cos_theta23 - L1 * cos_theta2 - L2 * cos_theta23) * cos_theta1;
    MAT_IDX(arm_state->jacobian, 0, 2) = (-H2 * sin_theta23 + H3 * cos_theta23 + L2 * cos_theta23) * cos_theta1;
    MAT_IDX(arm_state->jacobian, 1, 0) = -(H2 * cos_theta23 + H3 * sin_theta23 + L1 * sin_theta2 + L2 * sin_theta23) * cos_theta1;
    MAT_IDX(arm_state->jacobian, 1, 1) = (H2 * sin_theta23 - H3 * cos_theta23 - L1 * cos_theta2 - L2 * cos_theta23) * sin_theta1;
    MAT_IDX(arm_state->jacobian, 1, 2) = (-H2 * sin_theta23 + H3 * cos_theta23 + L2 * cos_theta23) * sin_theta1;
    MAT_IDX(arm_state->jacobian, 2, 0) = 0;
    MAT_IDX(arm_state->jacobian, 2, 1) = -H2 * cos_theta23 - H3 * sin_theta23 - L1 * sin_theta2 - L2 * sin_theta23;
    MAT_IDX(arm_state->jacobian, 2, 2) = H2 * cos_theta23 + H3 * sin_theta23 + L2 * sin_theta23;
}

void solve_wrist_joints(Arm_State_t *arm_state) {
    Mat *R13 = arm_state->buffer1_3x3;
    Mat *R46 = arm_state->buffer2_3x3;

    // get rotation mat from after joints 1-3
    mat_submatrix_buffer(arm_state->sim_transforms[3], 0, 0, R13);
    
    // get rotation for joints 4-6
    mat_mult_buffer(R13, arm_state->desired_orientation_mat, R46);

    // get euler angles from rotation matrix
    // note: our spherical wrist has roll-pitch-roll euler angles (ZY'Z'') convention
    VEC_IDX(arm_state->theta46_compute, 0) = atan2f(MAT_IDX(R46, 1, 2), MAT_IDX(R46, 0, 2));
    VEC_IDX(arm_state->theta46_compute, 1) = acosf(clamp(MAT_IDX(R46, 2, 2), -1.0, 1.0));
    VEC_IDX(arm_state->theta46_compute, 2) = atan2f(MAT_IDX(R46, 2, 1), -MAT_IDX(R46, 2, 0));
}

