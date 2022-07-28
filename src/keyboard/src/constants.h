#define M_PI 3.14159265358979323846

enum TeleopMode { FORWARD_KINEMATICS, INVERSE_KINEMATICS, CYLINDRICAL_KINEMATICS };

const int NUM_JOINTS = 6;

const bool debug_joints = true;

// robot arm dimensions (mm)
// reference: https://youtu.be/Sgsn2CM3bjY
const double r1 = 11.5;
const double r2 = 103.0;
const double r3 = 0.0; // or 0.000000000001
const double d1 = 98.0;
const double d3 = 0.0;
const double d4 = 141.0;
const double d6 = 93.0;

// french dude's numbers
// const double r1 = 47.5;
// const double r2 = 110.0;
// const double r3 = 26.0;
// const double d1 = 133.0;
// const double d3 = 0.0;
// const double d4 = 117.5;
// const double d6 = 28.0;

// const float INITIAL_JOINT_POS[NUM_JOINTS] = {-39.303490, 144.054657, -45.363953, 150.968018, 151.095444, 57.624413};
// const float INITIAL_JOINT_POS[NUM_JOINTS] = {1.710949, 0.0, 354.332153, 0.0, 22.0, 180.0};

// const float INITIAL_JOINT_POS[NUM_JOINTS] = {11.5, 0.000000, 436.332153, 0.000000, 0.383972, 0.141593};
// const float INITIAL_JOINT_ANGLES[NUM_JOINTS] = {0.0, 175.102737, -81.462234, -0.000079, 176.366211, -179.997620};
// const float JOINT_ANGLE_OFFSETS[NUM_JOINTS] = {90.0, -85.102737, 171.462234, 90.0, -86.366211, 269.99762};

// const float INITIAL_JOINT_POS[NUM_JOINTS] = {245.500000, -0.000006, 203.000000, -0.000003, 89.999992, 179.999985};
// const float INITIAL_JOINT_ANGLES[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// const float JOINT_ANGLE_OFFSETS[NUM_JOINTS] = {90.0, 90.0, 0.0, 90.0, 90.0, 90.0};

const float INITIAL_JOINT_POS[NUM_JOINTS] = {0.0, 0.0, 180.0, 0.0, 0.0, 0.0};
// const float INITIAL_JOINT_ANGLES[NUM_JOINTS] = {0.0, 51.474922, 92.892136, 0.0, 35.632942, 0.0};
const float INITIAL_JOINT_ANGLES[NUM_JOINTS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// const float JOINT_ANGLE_OFFSETS[NUM_JOINTS] = {90.0, -90.0, 0.0, 90.0, 0.0, 90.0};
// const float JOINT_ANGLE_OFFSETS_COEFF[NUM_JOINTS] = {1.0, -1.0, 1.0, 1.0, 1.0, 1.0};
