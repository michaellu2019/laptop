# define PI 3.14159265358979323846

# define NUM_ARM_SERVOS 6
# define NUM_JOINTS_PER_ARM 7
# define NUM_ARMS 2

enum TeleopMode { FORWARD_KINEMATICS, INVERSE_KINEMATICS, CYLINDRICAL_KINEMATICS };

const char LEFT[100] = "left";
const char RIGHT[100] = "right";

const float INITIAL_JOINT_ANGLES[NUM_ARMS * NUM_JOINTS_PER_ARM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

struct PoseOffset {
    float x;
    float y;
    float z;
    float a;
    float b;
    float c;
};

PoseOffset j_pose_offset = {-0.05, 0.0, 0.0, 0.0, 0.0, 0.0};
PoseOffset k_pose_offset = {-0.07, 0.0, -0.03, 0.0, 0.0, 0.0};

std::map<char, PoseOffset> keyboard_character_to_home_pose_offset = {
    {'j', j_pose_offset},
    {'k', k_pose_offset}
};