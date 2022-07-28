#include "VarSpeedServo.h"
#include "_constants.h"

#include "ros.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/JointState.h"


// Subscriber Callback to store the jointstate position values in the global variables
void servoControlSubscriberCallbackJointState(const sensor_msgs::JointState& msg) {
  for (int i = 0; i < NUM_ARM_SERVOS; i++) {
//    TARGET_JOINT_POSITIONS[i] = msg.position[i];
    right_arm.move_servo(i, msg.position[i] * arm_servo_joint_state_direction_vals[RIGHT_ARM][i] + arm_servo_joint_state_offset_vals[RIGHT_ARM][i], 30);
  }
  // Call the method to write the joint positions to the servo motors
//  writeServos();

  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN));
}

ros::NodeHandle node_handle;

ros::Subscriber<sensor_msgs::JointState> servo_control_subscriber_joint_state("controller_joint_states", &servoControlSubscriberCallbackJointState);

void setup() {
//  Serial.begin(9600);
//  Serial.println("Init...");
  
  node_handle.getHardware()->setBaud(115200);
  node_handle.initNode();
  node_handle.subscribe(servo_control_subscriber_joint_state);
  
  pinMode(LED_BUILTIN, OUTPUT);

  right_arm.initialize(30, 100);
}

void loop() {
  node_handle.spinOnce();
  delay(1);
}
