#define DEG2RAD (3.14159265358979f / 180.0f)
#define RAD2DEG (180.0f / 3.14159265358979f)

typedef struct ArmState {
    // shoulder + elbow
    float theta1;
    float theta2;

    // twist
    float theta3;

    // planar joints
    float theta4;
    float theta5;

    // gripper
    float theta6;
} ArmState;

float ARM_LIMITS[6][2] = {
    {0.0f, 180.0f}, // shoulder
    {0.0f, 180.0f}, // elbow
    {-90.0f, 90.0f}, // twist
    {0.0f, 180.0f}, // planar joint 1
    {0.0f, 180.0f}, // planar joint 2
    {0.0f, 90.0f} // gripper
};

float LINK_LENGTHS[5] = {
    0.1f,
    0.1f,
    0.0f,
    0.05f,
    0.05f
};

void translate_x(ArmState* state, float delta_x);
void translate_y(ArmState* state, float delta_y);
void translate_z(ArmState* state, float delta_z);
void roll_wrist(ArmState* state, float delta_theta3);
void pitch_wrist(ArmState* state, float delta_theta4);
void move_along_axis(ArmState* state, float delta_z);
void clamp_arm_state(ArmState* state);

