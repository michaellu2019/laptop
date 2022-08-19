#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/JointState.h"
#include "sstream"
#include "math.h"

#include "constants.h"

sensor_msgs::JointState joint_states;
std_msgs::Float32MultiArray controller_joint_states;
std_msgs::Int8 controller_joint_triggers;
std_msgs::Int8 controller_endeffector_triggers;

ros::Publisher controller_endeffector_triggers_pub;
ros::Publisher controller_joint_triggers_pub;
ros::Publisher controller_joint_states_pub;

void moveit_joint_states_callback(const sensor_msgs::JointState& moveit_joint_states) {
    for (int i = 0; i < NUM_ARMS * NUM_JOINTS_PER_ARM; i++) {
        if (i != NUM_JOINTS_PER_ARM - 1 && i != NUM_ARMS * NUM_JOINTS_PER_ARM - 1) {
            controller_joint_states.data[i] = moveit_joint_states.position[i] * 180.0/PI;
        } else {
            controller_joint_states.data[i] = moveit_joint_states.position[i] * -1000;
        }
    }
}

void moveit_joint_triggers_callback(const std_msgs::Int8& moveit_joint_triggers) {
    controller_joint_states_pub.publish(controller_joint_states);
}

void moveit_endeffector_triggers_callback(const std_msgs::Int8& moveit_endeffector_triggers) {
    controller_endeffector_triggers.data = moveit_endeffector_triggers.data;
    controller_endeffector_triggers_pub.publish(controller_endeffector_triggers);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_controller");

    ros::NodeHandle n;

    ros::Subscriber moveit_joint_states_sub = n.subscribe(JOINT_STATES_TOPIC, 1000, moveit_joint_states_callback);
    // ros::Subscriber moveit_joint_triggers_sub = n.subscribe(JOINT_TRIGGERS_TOPIC, 1000, moveit_joint_triggers_callback);
    ros::Subscriber moveit_endeffector_triggers_sub = n.subscribe(ENDEFFECTOR_TRIGGERS_TOPIC, 1000, moveit_endeffector_triggers_callback);

    controller_joint_states_pub = n.advertise<std_msgs::Float32MultiArray>(CONTROLLER_JOINT_STATES_TOPIC, 1000);
    controller_joint_triggers_pub = n.advertise<std_msgs::Int8>(CONTROLLER_JOINT_TRIGGERS_TOPIC, 1000);
    controller_endeffector_triggers_pub = n.advertise<std_msgs::Int8>(CONTROLLER_ENDEFFECTOR_TRIGGERS_TOPIC, 1000);

    ros::Rate loop_rate(30);
    
    controller_joint_states.data.resize(NUM_ARMS * NUM_JOINTS_PER_ARM);
    
    for (int i = 0; i < NUM_ARMS * NUM_JOINTS_PER_ARM; i++) {
        std::stringstream ss;
        if (i < NUM_JOINTS_PER_ARM) {
            ss << "link_l" << (i == NUM_JOINTS_PER_ARM - 1 ? 8 : i + 1);
        } else {
            ss << "link_r" << (i == NUM_ARMS * NUM_JOINTS_PER_ARM - 1 ? 8 : i - NUM_JOINTS_PER_ARM + 1);
        }
        
        ROS_INFO("Initial Joint %s at Position %f", ss.str().c_str(), controller_joint_states.data[i]);
    }   

    while (ros::ok()) {
        controller_joint_states_pub.publish(controller_joint_states);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
} 