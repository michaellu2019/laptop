#include "moveit/move_group_interface/move_group_interface.h"

#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"

#include "constants.h"
#include "movement.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_controller");

  ros::NodeHandle n;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP_LAPTOP_SCREEN = "laptop_screen_manipulator";
  static const std::string PLANNING_GROUP_LEFT_ARM = "left_arm_manipulator";
  static const std::string PLANNING_GROUP_LEFT_ENDEFFECTOR = "left_arm_endeffector";
  static const std::string PLANNING_GROUP_RIGHT_ARM = "right_arm_manipulator";
  static const std::string PLANNING_GROUP_RIGHT_ENDEFFECTOR = "right_arm_endeffector";
  
  moveit::planning_interface::MoveGroupInterface laptop_screen_move_group_interface(PLANNING_GROUP_LAPTOP_SCREEN);
  moveit::planning_interface::MoveGroupInterface left_arm_move_group_interface(PLANNING_GROUP_LEFT_ARM);
  moveit::planning_interface::MoveGroupInterface left_endeffector_move_group_interface(PLANNING_GROUP_LEFT_ENDEFFECTOR);
  moveit::planning_interface::MoveGroupInterface right_arm_move_group_interface(PLANNING_GROUP_RIGHT_ARM);
  moveit::planning_interface::MoveGroupInterface right_endeffector_move_group_interface(PLANNING_GROUP_RIGHT_ENDEFFECTOR);

  char latop_screen_initial_pose_name[100] = "laptop_screen_home";
  char left_arm_initial_pose_name[100] = "left_arm_home";
  char right_arm_initial_pose_name[100] = "right_arm_home";

  move_planning_group(&laptop_screen_move_group_interface, latop_screen_initial_pose_name);
  push_key(&left_endeffector_move_group_interface, LEFT);

  ros::Duration(2).sleep();

  move_planning_group(&left_arm_move_group_interface, left_arm_initial_pose_name);
  push_key(&left_endeffector_move_group_interface, LEFT);

  geometry_msgs::PoseStamped left_arm_home_pose = left_arm_move_group_interface.getCurrentPose("link_l6");
  // move_planning_group(&right_arm_move_group_interface, right_arm_initial_pose_name);
  // geometry_msgs::PoseStamped right_arm_home_pose = right_arm_move_group_interface.getCurrentPose("link_r6");

  geometry_msgs::PoseStamped target_pose1 = get_pose_from_keyboard_character('j', left_arm_home_pose);
  move_planning_group(&left_arm_move_group_interface, target_pose1);
  push_key(&left_endeffector_move_group_interface, LEFT);

  ros::Duration(3).sleep();

  geometry_msgs::PoseStamped target_pose2 = get_pose_from_keyboard_character('k', left_arm_home_pose);
  move_planning_group(&left_arm_move_group_interface, target_pose2);
  push_key(&left_endeffector_move_group_interface, LEFT);

  ros::Duration(3).sleep();

  move_planning_group(&left_arm_move_group_interface, left_arm_initial_pose_name);
  push_key(&left_endeffector_move_group_interface, LEFT);
  // move_planning_group(&right_arm_move_group_interface, right_arm_initial_pose_name);
  
  ros::Duration(3).sleep();

  ros::shutdown();
  return 0;
}