# define PI 3.14159265358979323846

# define NUM_ARM_SERVOS 6
# define NUM_JOINTS_PER_ARM 7
# define NUM_ARMS 2

# define NUM_KEYBOARD_ROWS 6

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

const float keyboard_character_offset_scale = 0.01;
const float keyboard_character_row_offsets[NUM_KEYBOARD_ROWS] = {3.0, 3.0, 3.8, 2.0, 2.1, 2.1};
const float left_arm_keyboard_character_column_start_offsets[NUM_KEYBOARD_ROWS] = {-5.0, -5.0, -5.6, -5.2, -5.6, -5.6};
const float left_arm_keyboard_character_column_spacing[NUM_KEYBOARD_ROWS] = {3.0, 3.0, 1.9, 1.9, 2.1, 2.1};
const float left_arm_keyboard_character_press_height_offsets[NUM_KEYBOARD_ROWS] = {-1.2, -1.2, -0.6, -0.8, -1.2, -1.2};
const float left_arm_keyboard_character_sag_compensation[NUM_KEYBOARD_ROWS] = {-5.0, -5.0, -0.14, -0.2, -5.6, -5.6};

// const int KEYBOARD_ROW_ONE = 0;
// const int KEYBOARD_ROW_TWO = 1;

// const int KEYBOARD_ROW_THREE = 2;
// PoseOffset left_t_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_THREE] + 0 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_THREE], keyboard_character_row_offsets[KEYBOARD_ROW_THREE], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_THREE] + 0 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_THREE], 0.0, 0.0, 0.0};
// PoseOffset left_y_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_THREE] + 1 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_THREE], keyboard_character_row_offsets[KEYBOARD_ROW_THREE], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_THREE] + 1 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_THREE], 0.0, 0.0, 0.0};
// PoseOffset left_u_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_THREE] + 2 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_THREE], keyboard_character_row_offsets[KEYBOARD_ROW_THREE], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_THREE] + 2 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_THREE], 0.0, 0.0, 0.0};
// PoseOffset left_i_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_THREE] + 3 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_THREE], keyboard_character_row_offsets[KEYBOARD_ROW_THREE], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_THREE] + 3 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_THREE], 0.0, 0.0, 0.0};
// PoseOffset left_o_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_THREE] + 4 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_THREE], keyboard_character_row_offsets[KEYBOARD_ROW_THREE], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_THREE] + 4 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_THREE], 0.0, 0.0, 0.0};
// PoseOffset left_p_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_THREE] + 5 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_THREE], keyboard_character_row_offsets[KEYBOARD_ROW_THREE], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_THREE] + 5 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_THREE], 0.0, 0.0, 0.0};
// PoseOffset left_left_bracket_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_THREE] + 6 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_THREE], keyboard_character_row_offsets[KEYBOARD_ROW_THREE], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_THREE] + 6 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_THREE], 0.0, 0.0, 0.0};
// PoseOffset left_right_bracket_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_THREE] + 7 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_THREE], keyboard_character_row_offsets[KEYBOARD_ROW_THREE], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_THREE] + 7 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_THREE], 0.0, 0.0, 0.0};
// PoseOffset left_backslash_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_THREE] + 8 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_THREE], keyboard_character_row_offsets[KEYBOARD_ROW_THREE], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_THREE] + 8 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_THREE], 0.0, 0.0, 0.0};

// const int KEYBOARD_ROW_FOUR = 3;
// PoseOffset left_g_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_FOUR] + 0 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_FOUR], keyboard_character_row_offsets[KEYBOARD_ROW_FOUR], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_FOUR] + 0 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_FOUR], 0.0, 0.0, 0.0};
// PoseOffset left_h_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_FOUR] + 1 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_FOUR], keyboard_character_row_offsets[KEYBOARD_ROW_FOUR], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_FOUR] + 1 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_FOUR], 0.0, 0.0, 0.0};
// PoseOffset left_j_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_FOUR] + 2 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_FOUR], keyboard_character_row_offsets[KEYBOARD_ROW_FOUR], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_FOUR] + 2 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_FOUR], 0.0, 0.0, 0.0};
// PoseOffset left_k_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_FOUR] + 3 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_FOUR], keyboard_character_row_offsets[KEYBOARD_ROW_FOUR], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_FOUR] + 3 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_FOUR], 0.0, 0.0, 0.0};
// PoseOffset left_l_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_FOUR] + 4 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_FOUR], keyboard_character_row_offsets[KEYBOARD_ROW_FOUR], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_FOUR] + 4 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_FOUR], 0.0, 0.0, 0.0};
// PoseOffset left_colon_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_FOUR] + 5 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_FOUR], keyboard_character_row_offsets[KEYBOARD_ROW_FOUR], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_FOUR] + 5 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_FOUR], 0.0, 0.0, 0.0};
// PoseOffset left_quote_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_FOUR] + 6 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_FOUR], keyboard_character_row_offsets[KEYBOARD_ROW_FOUR], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_FOUR] + 6 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_FOUR], 0.0, 0.0, 0.0};
// PoseOffset left_enter_pose_offset = {left_arm_keyboard_character_column_start_offsets[KEYBOARD_ROW_FOUR] + 7 * left_arm_keyboard_character_column_spacing[KEYBOARD_ROW_FOUR], keyboard_character_row_offsets[KEYBOARD_ROW_FOUR], left_arm_keyboard_character_press_height_offsets[KEYBOARD_ROW_FOUR] + 7 * left_arm_keyboard_character_sag_compensation[KEYBOARD_ROW_FOUR], 0.0, 0.0, 0.0};

// const int KEYBOARD_ROW_FIVE = 4;

std::vector<std::vector<std::string>> left_arm_keyboard_key_set {{/*"f5", "f6", "f7", "f8", "f9", "f10", "f11", "f12", "power", "delete"*/},
                                                                 {/*"5", "6", "7", "8", "9", "0", "-", "+", "backspace"*/},
                                                                 {"t", "y", "u", "i", "o", "p", "[", "]", "\\"},
                                                                 {"g", "h", "j", "k", "l", ";", "'", "enter"},
                                                                 {"b", "n", "m", ",", ".", "/", "shift"},
                                                                 {/*"space", "alt", "j", "k", "l", ";", "'", "enter"*/}};
                                                                       
std::map<std::string, PoseOffset> left_arm_keyboard_key_to_offset = {

    // {"t", left_t_pose_offset},
    // {"y", left_y_pose_offset},
    // {"u", left_u_pose_offset},
    // {"i", left_i_pose_offset},
    // {"o", left_o_pose_offset},
    // {"p", left_p_pose_offset},
    // {"[", left_left_bracket_pose_offset},
    // {"]", left_right_bracket_pose_offset},
    // {"\\", left_backslash_pose_offset},

    // {"g", left_g_pose_offset},
    // {"h", left_h_pose_offset},
    // {"j", left_j_pose_offset},
    // {"k", left_k_pose_offset},
    // {"l", left_l_pose_offset},
    // {";", left_colon_pose_offset},
    // {"'", left_quote_pose_offset},
    // {"enter", left_enter_pose_offset},
};

void setup_keyboard_key_to_offset_map() {
    for (int row = 0; row < left_arm_keyboard_key_set.size(); row++) {
        std::vector<std::string> left_arm_keyboard_row_key_set = left_arm_keyboard_key_set[row];
        for (int col = 0; col < left_arm_keyboard_row_key_set.size(); col++) {
            float x = left_arm_keyboard_character_column_start_offsets[row] + col * left_arm_keyboard_character_column_spacing[row];
            float y = keyboard_character_row_offsets[row];
            float z = left_arm_keyboard_character_press_height_offsets[row] + col * left_arm_keyboard_character_sag_compensation[row];
            PoseOffset left_arm_key_pose_offset = {x, y, z, 0.0, 0.0, 0.0};

            std::string key = left_arm_keyboard_row_key_set[col];
            left_arm_keyboard_key_to_offset[key] = left_arm_key_pose_offset;
        }
    }
}