#include "VarSpeedServo.h"
#include "_constants.h"

#include "ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/JointState.h"


// Subscriber Callback to store the jointstate position values in the global variables
void controller_joint_states_subscriber_callback(const sensor_msgs::JointState& msg) {
  for (int i = 0; i < NUM_ARMS * NUM_JOINTS_PER_ARM; i++) {
    const int servo_move_speed = 30;
    if (i < NUM_ARM_SERVOS) {
      left_arm.move_servo(i, msg.position[i] * arm_servo_joint_state_coefficient_vals[LEFT_ARM][i] + arm_servo_joint_state_offset_vals[LEFT_ARM][i], servo_move_speed);
    } else if (i >= NUM_JOINTS_PER_ARM) {
      int j = i - NUM_JOINTS_PER_ARM;
//      right_arm.move_servo(j, msg.position[j] * arm_servo_joint_state_coefficient_vals[RIGHT_ARM][j] + arm_servo_joint_state_offset_vals[RIGHT_ARM][j], servo_move_speed);
    } 

    if (i == NUM_JOINTS_PER_ARM - 1) {
      
    } else if (i == NUM_ARMS * NUM_JOINTS_PER_ARM - 1) {
      
    }
  }

  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN));
}

ros::NodeHandle node_handle;

ros::Subscriber<sensor_msgs::JointState> controller_joint_states_subscriber("controller_joint_states", &controller_joint_states_subscriber_callback);

void setup() {
//  Serial.begin(9600);
//  Serial.println("Init...");
  
  node_handle.getHardware()->setBaud(115200);
  node_handle.initNode();
  node_handle.subscribe(controller_joint_states_subscriber);
  
  pinMode(LED_BUILTIN, OUTPUT);

  left_arm.initialize(30, 100);
}

void loop() {
  node_handle.spinOnce();
  delay(1);
}
