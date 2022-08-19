#include "VarSpeedServo.h"
#include "_constants.h"

#include "ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/JointState.h"

// Subscriber Callback to store the jointstate position values in the global variables
void controller_joint_states_subscriber_callback(const std_msgs::Float32MultiArray& msg) {
  for (int i = 0; i < NUM_ARMS * NUM_JOINTS_PER_ARM; i++) {
    const int servo_move_speed = 15;
    if (i < NUM_ARM_SERVOS) {
      left_arm.move_servo(i, msg.data[i] * arm_servo_joint_state_coefficient_vals[LEFT_ARM][i] + arm_servo_joint_state_offset_vals[LEFT_ARM][i], servo_move_speed);
    } else if (i >= NUM_JOINTS_PER_ARM) {
      int j = i - NUM_JOINTS_PER_ARM;
//      right_arm.move_servo(j, msg.position[j] * arm_servo_joint_state_coefficient_vals[RIGHT_ARM][j] + arm_servo_joint_state_offset_vals[RIGHT_ARM][j], servo_move_speed);
    }
  }

//  left_endeffector.handle_pose_data(msg.data[NUM_JOINTS_PER_ARM - 1]);
}

void controller_endeffector_triggers_subscriber_callback(const std_msgs::Int8& msg) {
  digitalWrite(LED_BUILTIN, HIGH);
  if (msg.data == 0) {
//    left_endeffector.handle_pose_data(100.0);
    left_endeffector.open();
  }
}

ros::NodeHandle node_handle;

ros::Subscriber<std_msgs::Float32MultiArray> controller_joint_states_subscriber("controller_joint_states", &controller_joint_states_subscriber_callback);
ros::Subscriber<std_msgs::Int8> controller_endeffector_triggers_subscriber("controller_endeffector_triggers", &controller_endeffector_triggers_subscriber_callback);

void setup() {
//  Serial.begin(115200); 
//  Serial.println("Init...");
  
  node_handle.getHardware()->setBaud(115200);
  node_handle.initNode();
  node_handle.subscribe(controller_joint_states_subscriber);
  node_handle.subscribe(controller_endeffector_triggers_subscriber);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  left_arm.initialize(30, 100);
  left_endeffector.initialize();
//  left_endeffector.open();
}

void loop() {
  left_endeffector.tick();
  
  node_handle.spinOnce();
  delay(1);
}
