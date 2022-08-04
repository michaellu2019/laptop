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

geometry_msgs::PoseStamped get_pose_from_keyboard_character(char c, geometry_msgs::PoseStamped home_pose) {
  geometry_msgs::Vector3 home_orientation;
  tf2::Quaternion home_quaternion(home_pose.pose.orientation.x, home_pose.pose.orientation.y, home_pose.pose.orientation.z, home_pose.pose.orientation.w);
  tf2::Matrix3x3(home_quaternion).getRPY(home_orientation.x, home_orientation.y, home_orientation.z);
  
  geometry_msgs::PoseStamped keyboard_character_pose = create_pose_stamped(
    home_pose.pose.position.x + keyboard_character_to_home_pose_offset[c].x, 
    home_pose.pose.position.y + keyboard_character_to_home_pose_offset[c].y, 
    home_pose.pose.position.z + keyboard_character_to_home_pose_offset[c].z,
    home_orientation.x + keyboard_character_to_home_pose_offset[c].a, 
    home_orientation.y + keyboard_character_to_home_pose_offset[c].b, 
    home_orientation.z + keyboard_character_to_home_pose_offset[c].c
  );

  return keyboard_character_pose;
}

void move_arm(moveit::planning_interface::MoveGroupInterface* move_group_interface_arm, char* target_pose_name) {
  move_group_interface_arm->setJointValueTarget(move_group_interface_arm->getNamedTargetValues(target_pose_name));

  moveit::planning_interface::MoveGroupInterface::Plan plan_arm;
  bool success = (move_group_interface_arm->plan(plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  geometry_msgs::PoseStamped target_pose = move_group_interface_arm->getCurrentPose("link_6");
  ROS_INFO_NAMED("movement", "Moving arm to %s: %s", target_pose_name, success ? "" : "FAILED");
  log_pose(target_pose_name, target_pose.pose);

  move_group_interface_arm->move();
}

void move_arm(moveit::planning_interface::MoveGroupInterface* move_group_interface_arm, geometry_msgs::PoseStamped target_pose) {
  move_group_interface_arm->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan_arm;
  bool success = (move_group_interface_arm->plan(plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  char target_pose_name[100] = "target_pose";
  ROS_INFO_NAMED("movement", "Moving arm to %s. %s", target_pose_name, success ? "" : "FAILED");
  log_pose(target_pose_name, target_pose.pose);

  move_group_interface_arm->move();
}