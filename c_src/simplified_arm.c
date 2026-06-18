#include "math.h"
#include "simplified_arm.h"

/* ── clamp ──────────────────────────────────────────────────────────────── */

void clamp_arm_state(ArmState* state) {
    float* angles = &state->theta1;
    for (int i = 0; i < 6; i++) {
        if (angles[i] < ARM_LIMITS[i][0]) angles[i] = ARM_LIMITS[i][0];
        if (angles[i] > ARM_LIMITS[i][1]) angles[i] = ARM_LIMITS[i][1];
    }
}

/* ── 2DOF planar IK ─────────────────────────────────────────────────────── */
/*
 * Solves a 2-link arm in the plane to reach (r, z).
 * r = radial/horizontal reach, z = vertical.
 * Writes result into *t1_out, *t2_out (degrees).
 * Returns 1 on success, 0 if target is out of reach.
 */
static int planar_ik(float r, float z, float l1, float l2,
                     float* t1_out, float* t2_out)
{
    float dist2 = r * r + z * z;
    float dist  = sqrtf(dist2);

    float reach_max = l1 + l2;
    float reach_min = fabsf(l1 - l2);
    if (dist > reach_max || dist < reach_min)
        return 0;

    /* elbow angle via law of cosines */
    float cos_t2 = (dist2 - l1 * l1 - l2 * l2) / (2.0f * l1 * l2);
    if (cos_t2 >  1.0f) cos_t2 =  1.0f;
    if (cos_t2 < -1.0f) cos_t2 = -1.0f;
    float t2 = acosf(cos_t2);

    /* shoulder angle */
    float k1 = l1 + l2 * cosf(t2);
    float k2 = l2 * sinf(t2);
    float t1 = atan2f(r, z) - atan2f(k2, k1);

    *t1_out = t1 * RAD2DEG;
    *t2_out = t2 * RAD2DEG;
    return 1;
}

/* ── shoulder/elbow plane (fixed XZ) ────────────────────────────────────── */

void translate_x(ArmState* state, float delta_x) {
    float L1 = LINK_LENGTHS[0];
    float L2 = LINK_LENGTHS[1];
    float t1  = state->theta1 * DEG2RAD;
    float t2  = state->theta2 * DEG2RAD;

    /* current end of the shoulder/elbow chain */
    float r = L1 * sinf(t1) + L2 * sinf(t1 + t2);
    float z = L1 * cosf(t1) + L2 * cosf(t1 + t2);

    r += delta_x;

    float new_t1, new_t2;
    if (planar_ik(r, z, L1, L2, &new_t1, &new_t2)) {
        state->theta1 = new_t1;
        state->theta2 = new_t2;
    }
    clamp_arm_state(state);
}

void translate_z(ArmState* state, float delta_z) {
    float L1 = LINK_LENGTHS[0];
    float L2 = LINK_LENGTHS[1];
    float t1  = state->theta1 * DEG2RAD;
    float t2  = state->theta2 * DEG2RAD;

    float r = L1 * sinf(t1) + L2 * sinf(t1 + t2);
    float z = L1 * cosf(t1) + L2 * cosf(t1 + t2);

    z += delta_z;

    float new_t1, new_t2;
    if (planar_ik(r, z, L1, L2, &new_t1, &new_t2)) {
        state->theta1 = new_t1;
        state->theta2 = new_t2;
    }
    clamp_arm_state(state);
}

void translate_y(ArmState* state, float delta_y) {
    /* Y has no DOF in this arm configuration — no-op */
    (void)state;
    (void)delta_y;
}

/* ── roll ───────────────────────────────────────────────────────────────── */

void roll_wrist(ArmState* state, float delta_theta3) {
    state->theta3 += delta_theta3;
    clamp_arm_state(state);
}

/* ── wrist pitch ─────────────────────────────────────────────────────────── */

void pitch_wrist(ArmState* state, float delta_theta4) {
    state->theta4 += delta_theta4;
    clamp_arm_state(state);
}

/* ── move along axis ─────────────────────────────────────────────────────── */
/*
 * The wrist 2DOF chain lives in the plane defined by theta3.
 * theta5 is the cumulative pointing angle of the end effector in that plane,
 * measured from the plane's "up" axis (same convention as theta1/theta2).
 *
 * The axis direction in the wrist plane:
 *   dir_r =  sin(t4 + t5)   (radial within the wrist plane)
 *   dir_z =  cos(t4 + t5)   (vertical)
 *
 * Advancing delta along this axis moves the tip by (dr, dz) in the wrist
 * plane. We then back-solve theta4,theta5 to the new tip position, and also
 * back-solve theta1,theta2 to absorb the change in where the wrist base sits
 * (since the wrist base = end of the shoulder/elbow chain, and that hasn't
 * moved — only the wrist tip moves, so theta1/theta2 stay put).
 *
 * Wait — on reflection: the shoulder/elbow base is fixed. The wrist base is
 * the end of the shoulder/elbow chain and is also fixed (theta1/theta2 didn't
 * change). Moving along the wrist axis only changes theta4/theta5.
 *
 * However, the axis direction also involves theta1+theta2 in world space if
 * we want to express it globally. Since the wrist plane is defined by theta3,
 * and the wrist joints live in that plane, we only need to re-solve the wrist
 * 2DOF chain (theta4, theta5) for the new tip position in that plane.
 *
 * If the new tip is out of the wrist chain's reach we do nothing.
 */
void move_along_axis(ArmState* state, float delta) {
    float L3 = LINK_LENGTHS[3];
    float L4 = LINK_LENGTHS[4];
    float t4  = state->theta4 * DEG2RAD;
    float t5  = state->theta5 * DEG2RAD;

    /* current wrist tip in the wrist plane (r, z) */
    float r = L3 * sinf(t4) + L4 * sinf(t4 + t5);
    float z = L3 * cosf(t4) + L4 * cosf(t4 + t5);

    /* axis direction: the angle the last link points along */
    float axis_angle = t4 + t5;
    float dir_r = sinf(axis_angle);
    float dir_z = cosf(axis_angle);

    r += delta * dir_r;
    z += delta * dir_z;

    /* re-solve wrist chain; shoulder/elbow unchanged */
    float new_t4, new_t5;
    if (planar_ik(r, z, L3, L4, &new_t4, &new_t5)) {
        /*
         * planar_ik returns elbow-down. We want to preserve the axis
         * direction after the move, so check both elbow configurations
         * and pick whichever keeps (t4+t5) closest to the original axis.
         */
        float axis_orig = (state->theta4 + state->theta5) * DEG2RAD;

        float axis_a = (new_t4 + new_t5) * DEG2RAD;
        float diff_a = fabsf(axis_a - axis_orig);

        /* elbow-up solution: t2_up = -t2, t1_up adjusted */
        float t5_up  = -new_t5 * DEG2RAD;
        float k1 = L3 + L4 * cosf(t5_up);
        float k2 = L4 * sinf(t5_up);
        float t4_up = atan2f(r, z) - atan2f(k2, k1);
        float axis_b = t4_up + t5_up;
        float diff_b = fabsf(axis_b - axis_orig);

        if (diff_b < diff_a) {
            state->theta4 = t4_up * RAD2DEG;
            state->theta5 = t5_up * RAD2DEG;
        } else {
            state->theta4 = new_t4;
            state->theta5 = new_t5;
        }
    }
    clamp_arm_state(state);
}