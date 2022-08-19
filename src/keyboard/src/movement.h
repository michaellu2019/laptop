#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

void pose_to_string(char* pose_str, geometry_msgs::Pose pose) {
  geometry_msgs::Vector3 orientation;
  tf2::Quaternion quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3(quaternion).getRPY(orientation.x, orientation.y, orientation.z);
  sprintf(pose_str, "(%f, %f, %f, %f, %f, %f)", pose.position.x, pose.position.y, pose.position.z, orientation.x, orientation.y, orientation.z);
}

void log_pose(char* pose_name, geometry_msgs::Pose pose) {
  char pose_str[100];
  pose_to_string(pose_str, pose);
  ROS_INFO_NAMED("pose", "%s: %s", pose_name, pose_str);
}

geometry_msgs::PoseStamped create_pose_stamped(float x, float y, float z, float a, float b, float c) {
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "world";
  target_pose.header.stamp = ros::Time::now();

  tf2::Quaternion target_quaternion;
  target_quaternion.setRPY(a, b, c);
  target_pose.pose.orientation.x = target_quaternion.getX();
  target_pose.pose.orientation.y = target_quaternion.getY();
  target_pose.pose.orientation.z = target_quaternion.getZ();
  target_pose.pose.orientation.w = target_quaternion.getW();

  target_pose.pose.position.x = x;
  target_pose.pose.position.y = y;
  target_pose.pose.position.z = z;

  return target_pose;
}

geometry_msgs::PoseStamped get_pose_from_keyboard_key(std::string c, geometry_msgs::PoseStamped home_pose, const char* side) {
  geometry_msgs::Vector3 home_orientation;
  tf2::Quaternion home_quaternion(home_pose.pose.orientation.x, home_pose.pose.orientation.y, home_pose.pose.orientation.z, home_pose.pose.orientation.w);
  tf2::Matrix3x3(home_quaternion).getRPY(home_orientation.x, home_orientation.y, home_orientation.z);
  
  geometry_msgs::PoseStamped keyboard_key_pose = create_pose_stamped(
    home_pose.pose.position.x + keyboard_key_offset_position_scale * left_arm_keyboard_key_to_offset[c].x, 
    home_pose.pose.position.y + keyboard_key_offset_position_scale * left_arm_keyboard_key_to_offset[c].y, 
    home_pose.pose.position.z + keyboard_key_offset_position_scale * left_arm_keyboard_key_to_offset[c].z,
    home_orientation.x + left_arm_keyboard_key_to_offset[c].a, 
    home_orientation.y + left_arm_keyboard_key_to_offset[c].b, 
    home_orientation.z + left_arm_keyboard_key_to_offset[c].c
  );

  return keyboard_key_pose;
}

void trigger_joint_publisher(ros::Publisher* joint_triggers_pub, const char* side) {
  int i = side == RIGHT;
  joint_triggers.data = side == RIGHT;
  joint_triggers_pub->publish(joint_triggers);
}

void move_planning_group(ros::Publisher* joint_triggers_pub, moveit::planning_interface::MoveGroupInterface* move_group_interface, char* target_pose_name) {
  move_group_interface->setJointValueTarget(move_group_interface->getNamedTargetValues(target_pose_name));

  moveit::planning_interface::MoveGroupInterface::Plan plan_arm;
  bool success = (move_group_interface->plan(plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  move_group_interface->move();

  geometry_msgs::PoseStamped target_pose = move_group_interface->getCurrentPose("link_l6");
  ROS_INFO_NAMED("movement", "Moving arm to %s: %s", target_pose_name, success ? "" : "FAILED");
  log_pose(target_pose_name, target_pose.pose);

  trigger_joint_publisher(joint_triggers_pub, move_group_interface->getName() == PLANNING_GROUP_LEFT_ARM ? LEFT : RIGHT);
}

void move_planning_group(ros::Publisher* joints_trigger_pub, moveit::planning_interface::MoveGroupInterface* move_group_interface, geometry_msgs::PoseStamped target_pose) {
  move_group_interface->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan_arm;
  bool success = (move_group_interface->plan(plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  char target_pose_name[100] = "target_pose";
  ROS_INFO_NAMED("movement", "Moving arm to %s. %s", target_pose_name, success ? "" : "FAILED");
  log_pose(target_pose_name, target_pose.pose);

  move_group_interface->move();

  trigger_joint_publisher(joints_trigger_pub, move_group_interface->getName() == PLANNING_GROUP_LEFT_ARM ? LEFT : RIGHT);
}

void push_key(ros::Publisher* endeffector_triggers_pub, moveit::planning_interface::MoveGroupInterface* move_group_interface, const char* side) {
  char target_pose[100];
  sprintf(target_pose, "%s_endeffector_open", side);
  int i = side == RIGHT;
  endeffector_triggers.data = side == RIGHT;
  endeffector_triggers_pub->publish(endeffector_triggers);
  // move_planning_group(move_group_interface, target_pose);
  // sprintf(target_pose, "%s_endeffector_closed", side);
  // move_planning_group(move_group_interface, target_pose);
}

void test_keys(ros::Publisher* joint_triggers_pub, ros::Publisher* endeffector_triggers_pub, moveit::planning_interface::MoveGroupInterface* move_group_interface, geometry_msgs::PoseStamped home_pose, const char* side) {
  std::map<std::string, PoseOffset> keyboard_key_to_offset = left_arm_keyboard_key_to_offset;
  if (side == RIGHT) {
    keyboard_key_to_offset = right_arm_keyboard_key_to_offset;
  }
  
  
  int NUM_TESTS = left_arm_keyboard_key_to_offset.size();
  std::map<std::string, PoseOffset>::iterator it;
  int i = 0;

  for (it = left_arm_keyboard_key_to_offset.begin(); it != left_arm_keyboard_key_to_offset.end(); it++) {
    i++;
    if (i > NUM_TESTS)
      break;

    std::string test_str = it->first;
    ROS_INFO_NAMED("test", "Testing %s", test_str.c_str());
    geometry_msgs::PoseStamped target_pose1 = get_pose_from_keyboard_key(it->first, home_pose, side);
    move_planning_group(joint_triggers_pub, move_group_interface, target_pose1);
    push_key(endeffector_triggers_pub, move_group_interface, LEFT);
    ros::Duration(1).sleep();
  }
}