#include "std_msgs/Int8.h"

# define PI 3.14159265358979323846

# define NUM_ARM_SERVOS 6
# define NUM_JOINTS_PER_ARM 7
# define NUM_ARMS 2

# define NUM_KEYBOARD_ROWS 6

const char LEFT[100] = "left";
const char RIGHT[100] = "right";

const char JOINT_STATES_TOPIC[100] = "joint_states";
const char JOINT_TRIGGERS_TOPIC[100] = "joint_triggers";
const char ENDEFFECTOR_TRIGGERS_TOPIC[100] = "endeffector_triggers";
const char CONTROLLER_JOINT_STATES_TOPIC[100] = "controller_joint_states";
const char CONTROLLER_JOINT_TRIGGERS_TOPIC[100] = "controller_joint_triggers";
const char CONTROLLER_ENDEFFECTOR_TRIGGERS_TOPIC[100] = "controller_endeffector_triggers";

struct PoseOffset {
    float x;
    float y;
    float z;
    float a;
    float b;
    float c;
};

const float keyboard_key_offset_position_scale = 0.01;
const float keyboard_key_offset_orientation_scales[NUM_KEYBOARD_ROWS] = {-0.25, -0.1, 0.0, 0.0, 0.0, 0.0};
const float keyboard_key_row_offsets[NUM_KEYBOARD_ROWS] = {5.7, 4.7, 3.6, 2.0, 0.2, -1.2};
const float left_arm_keyboard_key_column_start_offsets[NUM_KEYBOARD_ROWS] = {-3.4, -4.0, -5.6, -5.4, -4.5, -0.2};
const float left_arm_keyboard_key_column_spacing[NUM_KEYBOARD_ROWS] = {1.9, 1.9, 1.9, 1.9, 1.9, 1.9};
// const float left_arm_keyboard_key_press_height_offsets[NUM_KEYBOARD_ROWS] = {-0.2, 0.2, -0.5, -0.8, -0.8, -1.2};
const float left_arm_keyboard_key_press_height_offsets[NUM_KEYBOARD_ROWS] = {0.7, 1.3, 0.6, 0.4, 0.3, -0.25};
const float left_arm_keyboard_key_sag_compensation[NUM_KEYBOARD_ROWS] = {-0.08, -0.14, -0.14, -0.24, -0.25, -0.15};

PoseOffset test_pose_offset = {0.0, 0.0, 0.0, 0.0, 0.5, 0.0};

std::vector<std::vector<std::string>> left_arm_keyboard_key_set {{"f7", "f8", "f9", "f10", "f11", "f12", "power", "delete"},
                                                                 {"6", "7", "8", "9", "0", "-", "+", "backspace"},
                                                                 {"t", "y", "u", "i", "o", "p", "[", "]", "\\"},
                                                                 {"g", "h", "j", "k", "l", ";", "'", "enter"},
                                                                 {"b", "n", "m", ",", ".", "/", "shift"},
                                                                 {"space", "alt", "options", "left_arrow", "up_down_arrows", "right_arrow"}};
                                                                       
std::map<std::string, PoseOffset> left_arm_keyboard_key_to_offset = {};
std::map<std::string, PoseOffset> right_arm_keyboard_key_to_offset = {};

void setup_keyboard_key_to_offset_map() {
    for (int row = 0; row < left_arm_keyboard_key_set.size(); row++) {
        std::vector<std::string> left_arm_keyboard_row_key_set = left_arm_keyboard_key_set[row];
        for (int col = 0; col < left_arm_keyboard_row_key_set.size(); col++) {
            std::string key = left_arm_keyboard_row_key_set[col];

            float x = left_arm_keyboard_key_column_start_offsets[row] + col * left_arm_keyboard_key_column_spacing[row];
            float y = keyboard_key_row_offsets[row];
            float z = left_arm_keyboard_key_press_height_offsets[row] + col * left_arm_keyboard_key_sag_compensation[row];

            float roll = 0.0;
            float pitch = key == "right_arrow" ? 0.2 : 0.0;
            float yaw = keyboard_key_offset_orientation_scales[row] * atan2(y, x);
            PoseOffset left_arm_key_pose_offset = {x, y, z, roll, pitch, yaw};

            left_arm_keyboard_key_to_offset[key] = left_arm_key_pose_offset;
        }
    }
}

std_msgs::Int8 endeffector_triggers;
std_msgs::Int8 joint_triggers;

static const std::string PLANNING_GROUP_LAPTOP_SCREEN = "laptop_screen_manipulator";
static const std::string PLANNING_GROUP_LEFT_ARM = "left_arm_manipulator";
static const std::string PLANNING_GROUP_LEFT_ENDEFFECTOR = "left_arm_endeffector";
static const std::string PLANNING_GROUP_RIGHT_ARM = "right_arm_manipulator";
static const std::string PLANNING_GROUP_RIGHT_ENDEFFECTOR = "right_arm_endeffector";

char latop_screen_initial_pose_name[100] = "laptop_screen_home";
char left_arm_initial_pose_name[100] = "left_arm_home";
char right_arm_initial_pose_name[100] = "right_arm_home";