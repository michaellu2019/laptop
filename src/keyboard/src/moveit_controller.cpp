#include "moveit/move_group_interface/move_group_interface.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "math.h"

#include "constants.h"
#include "movement.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "moveit_controller");

  ros::NodeHandle n;

  ros::Publisher endeffector_triggers_pub = n.advertise<std_msgs::Int8>(ENDEFFECTOR_TRIGGERS_TOPIC, 1000);
  ros::Publisher joint_triggers_pub = n.advertise<std_msgs::Int8>(JOINT_TRIGGERS_TOPIC, 1000);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  setup_keyboard_key_to_offset_map();
  
  moveit::planning_interface::MoveGroupInterface laptop_screen_move_group_interface(PLANNING_GROUP_LAPTOP_SCREEN);
  moveit::planning_interface::MoveGroupInterface left_arm_move_group_interface(PLANNING_GROUP_LEFT_ARM);
  moveit::planning_interface::MoveGroupInterface left_endeffector_move_group_interface(PLANNING_GROUP_LEFT_ENDEFFECTOR);
  moveit::planning_interface::MoveGroupInterface right_arm_move_group_interface(PLANNING_GROUP_RIGHT_ARM);
  moveit::planning_interface::MoveGroupInterface right_endeffector_move_group_interface(PLANNING_GROUP_RIGHT_ENDEFFECTOR);

  // move_planning_group(&laptop_screen_move_group_interface, latop_screen_initial_pose_name);
  // push_key(&left_endeffector_move_group_interface, LEFT);

  // ros::Duration(2).sleep();

  move_planning_group(&joint_triggers_pub, &left_arm_move_group_interface, left_arm_initial_pose_name);
  push_key(&endeffector_triggers_pub, &left_endeffector_move_group_interface, LEFT);
  geometry_msgs::PoseStamped left_arm_home_pose = left_arm_move_group_interface.getCurrentPose("link_l6");

  // move_planning_group(&joint_triggers_pub, &right_arm_move_group_interface, right_arm_initial_pose_name);
  // geometry_msgs::PoseStamped right_arm_home_pose = right_arm_move_group_interface.getCurrentPose("link_r6");

  // test_keys(&left_arm_move_group_interface, left_arm_home_pose, LEFT);

  // std::vector<std::string> key_set = {"h", "i"};
  // std::vector<std::string> key_set = {"power", "j", "enter"};
  // std::vector<std::string> key_set = {"power", "right_arrow", "enter"};
  // std::vector<std::string> key_set = {"l", "o", "l"};
  std::vector<std::string> key_set = {"l", "o", "l", "space", "h", "i"};

  for (int i = 0; i < key_set.size(); i++) {
    std::string key = key_set[i];
    ROS_INFO_NAMED("test", "Pushing %s", key.c_str());

    geometry_msgs::PoseStamped target_pose1 = get_pose_from_keyboard_key(key, left_arm_home_pose, LEFT);
    move_planning_group(&joint_triggers_pub, &left_arm_move_group_interface, target_pose1);
    push_key(&endeffector_triggers_pub, &left_endeffector_move_group_interface, LEFT);
    // ros::Duration(1).sleep();
  }

  move_planning_group(&joint_triggers_pub, &left_arm_move_group_interface, left_arm_initial_pose_name);
  // push_key(&left_endeffector_move_group_interface, LEFT);
  // move_planning_group(&right_arm_move_group_interface, right_arm_initial_pose_name);
  
  ros::Duration(1).sleep();

  ros::shutdown();
  return 0;
}