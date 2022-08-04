#include "moveit/move_group_interface/move_group_interface.h"

#include "constants.h"
#include "movement.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_controller");

  ros::NodeHandle n;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP_ARM = "manipulator";
  // static const std::string PLANNING_GROUP_GRIPPER = "gripper";
  
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
  // moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

  char initial_pose_name[100] = "home";
  move_arm(&move_group_interface_arm, initial_pose_name);

  geometry_msgs::PoseStamped home_pose = move_group_interface_arm.getCurrentPose("link_6");
  geometry_msgs::Vector3 home_orientation;
  tf2::Quaternion home_quaternion(home_pose.pose.orientation.x, home_pose.pose.orientation.y, home_pose.pose.orientation.z, home_pose.pose.orientation.w);
  tf2::Matrix3x3(home_quaternion).getRPY(home_orientation.x, home_orientation.y, home_orientation.z);

  ros::Duration(3).sleep();

  geometry_msgs::PoseStamped target_pose1 = get_pose_from_keyboard_character('j', home_pose);
  move_arm(&move_group_interface_arm, target_pose1);
  ros::Duration(3).sleep();

  geometry_msgs::PoseStamped target_pose2 = get_pose_from_keyboard_character('k', home_pose);
  move_arm(&move_group_interface_arm, target_pose2);
  ros::Duration(3).sleep();

  move_arm(&move_group_interface_arm, initial_pose_name);

  ros::shutdown();
  return 0;
}