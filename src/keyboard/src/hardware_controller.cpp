#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "sstream"
#include "math.h"

#include "constants.h"

sensor_msgs::JointState initial_joint_states;
sensor_msgs::JointState joint_states;
sensor_msgs::JointState arm_state;

void moveit_joint_states_callback(const sensor_msgs::JointState& moveit_joint_states) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        joint_states.position[i] = moveit_joint_states.position[i] * 180.0/M_PI;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_controller");

    ros::NodeHandle n;

    ros::Publisher controller_joint_states_pub = n.advertise<sensor_msgs::JointState>("controller_joint_states", 1000);
    // ros::Subscriber sub = n.subscribe("arm_teleop", 1000, armTeleopCallback);
    ros::Subscriber moveit_joint_states_sub = n.subscribe("joint_states", 1000, moveit_joint_states_callback);

    ros::Rate loop_rate(30);

    int count = 0;

    initial_joint_states.name.resize(NUM_JOINTS);
    initial_joint_states.position.resize(NUM_JOINTS);
    initial_joint_states.velocity.resize(NUM_JOINTS);
    initial_joint_states.effort.resize(NUM_JOINTS);

    joint_states.name.resize(NUM_JOINTS);
    joint_states.position.resize(NUM_JOINTS);
    joint_states.velocity.resize(NUM_JOINTS);
    joint_states.effort.resize(NUM_JOINTS);

    arm_state.name.resize(NUM_JOINTS);
    arm_state.position.resize(NUM_JOINTS);
    arm_state.velocity.resize(NUM_JOINTS);
    arm_state.effort.resize(NUM_JOINTS);

    float initial_joint_angles[NUM_JOINTS] = {INITIAL_JOINT_ANGLES[0], INITIAL_JOINT_ANGLES[1], INITIAL_JOINT_ANGLES[2], INITIAL_JOINT_ANGLES[3], INITIAL_JOINT_ANGLES[4], INITIAL_JOINT_ANGLES[5]};
    float initial_joint_pos[NUM_JOINTS] = {INITIAL_JOINT_POS[0], INITIAL_JOINT_POS[1], INITIAL_JOINT_POS[2], INITIAL_JOINT_POS[3], INITIAL_JOINT_POS[4], INITIAL_JOINT_POS[5]};
    // InverseK(initial_joint_pos, initial_joint_angles);
    for (int i = 0; i < NUM_JOINTS; i++) {
        std::stringstream ss;
        ss << "J" << i;

        initial_joint_states.name[i] = ss.str();
        initial_joint_states.position[i] = initial_joint_angles[i];
        // initial_joint_states.position[i] = 0;
        initial_joint_states.velocity[i] = 0;
        initial_joint_states.effort[i] = 0;

        joint_states.name[i] = initial_joint_states.name[i];
        joint_states.position[i] = initial_joint_states.position[i];
        joint_states.velocity[i] = initial_joint_states.velocity[i];
        joint_states.effort[i] = initial_joint_states.effort[i];

        arm_state.name[i] = i == 0 ? "X" : i == 1 ? "Y" : i == 2 ? "Z" : 
                            i == 3 ? "A" : i == 4 ? "B" : "C";
        arm_state.position[i] = initial_joint_pos[i];
        // arm_state.position[i] = 0;
        arm_state.velocity[i] = initial_joint_states.velocity[i];
        arm_state.effort[i] = initial_joint_states.effort[i];

        ROS_INFO("Initial Joint %s at Position %f", joint_states.name[i].c_str(), initial_joint_angles[i]);
    }   

    while (ros::ok()) {
        controller_joint_states_pub.publish(joint_states);
        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
} 