#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "dynamixel_workbench_msgs/DynamixelState.h"

#define _USE_MATH_DEFINES
#include <cmath>

#define DXL_LEFT	  1
#define DXL_RIGHT	  2
#define	ADDRESS		"Goal_Position"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_and_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle n;

  ros::ServiceClient goal_dynamixel_command_client_ = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("dynamixel_workbench/dynamixel_command");

  ros::Rate loop_rate(10);

  moveit::planning_interface::MoveGroupInterface arm("gluon");

  arm.setGoalJointTolerance(0.001);

  arm.setMaxAccelerationScalingFactor(1.0);
  arm.setMaxVelocityScalingFactor(1.0);

  //set the position of gripper, values refer to DynamixelWizard 2.0 --> goal position
  //set gripper open position
  dynamixel_workbench_msgs::DynamixelCommand open;
  open.request.id_1 = DXL_LEFT;
  open.request.id_2 = DXL_RIGHT;
  open.request.addr_name_1 = ADDRESS;
  open.request.addr_name_2 = ADDRESS;
  open.request.value_1 = 330;
  open.request.value_2 = 730;

  //set gripper release position
  dynamixel_workbench_msgs::DynamixelCommand release;
  release.request.id_1 = DXL_LEFT;
  release.request.id_2 = DXL_RIGHT;
  release.request.addr_name_1 = ADDRESS;
  release.request.addr_name_2 = ADDRESS;
  release.request.value_1 = 470;
  release.request.value_2 = 550;

  //set gripper close position
  dynamixel_workbench_msgs::DynamixelCommand close_mid;
  close_mid.request.id_1 = DXL_LEFT;
  close_mid.request.id_2 = DXL_RIGHT;
  close_mid.request.addr_name_1 = ADDRESS;
  close_mid.request.addr_name_2 = ADDRESS;
  close_mid.request.value_1 = 510;
  close_mid.request.value_2 = 510;

  //set gripper close position
  dynamixel_workbench_msgs::DynamixelCommand close;
  close.request.id_1 = DXL_LEFT;
  close.request.id_2 = DXL_RIGHT;
  close.request.addr_name_1 = ADDRESS;
  close.request.addr_name_2 = ADDRESS;
  close.request.value_1 = 542;
  close.request.value_2 = 482;

  //set the waypoints
  //in order of {joint1,joint2,joint3,joint4,joint5,joint6}, in degree
  //look_table position
  double look_table_axis1_deg = 0;
  double look_table_axis2_deg = 0;
  double look_table_axis3_deg = 0;
  double look_table_axis4_deg = 0;
  double look_table_axis5_deg = 0;
  double look_table_axis6_deg = 0;

  //(1) positions for cube
  //approach_pick_1
  double approach_pick_1_axis1_deg = 0;
  double approach_pick_1_axis2_deg = 0;
  double approach_pick_1_axis3_deg = 0;
  double approach_pick_1_axis4_deg = 0;
  double approach_pick_1_axis5_deg = 0;
  double approach_pick_1_axis6_deg = 0;

  //pick_1
  double pick_1_axis1_deg = 0;
  double pick_1_axis2_deg = 0;
  double pick_1_axis3_deg = 0;
  double pick_1_axis4_deg = 0;
  double pick_1_axis5_deg = 0;
  double pick_1_axis6_deg = 0;

  //(2) positions for triangular prism
  //approach_pick_2
  double approach_pick_2_axis1_deg = 0;
  double approach_pick_2_axis2_deg = 0;
  double approach_pick_2_axis3_deg = 0;
  double approach_pick_2_axis4_deg = 0;
  double approach_pick_2_axis5_deg = 0;
  double approach_pick_2_axis6_deg = 0;

  //pick_2
  double pick_2_axis1_deg = 0;
  double pick_2_axis2_deg = 0;
  double pick_2_axis3_deg = 0;
  double pick_2_axis4_deg = 0;
  double pick_2_axis5_deg = 0;
  double pick_2_axis6_deg = 0;

  //(3) positions for triangular prism
  //approach_pick_3
  double approach_pick_3_axis1_deg = 0;
  double approach_pick_3_axis2_deg = 0;
  double approach_pick_3_axis3_deg = 0;
  double approach_pick_3_axis4_deg = 0;
  double approach_pick_3_axis5_deg = 0;
  double approach_pick_3_axis6_deg = 0;

  //pick_3
  double pick_3_axis1_deg = 0;
  double pick_3_axis2_deg = 0;
  double pick_3_axis3_deg = 0;
  double pick_3_axis4_deg = 0;
  double pick_3_axis5_deg = 0;
  double pick_3_axis6_deg = 0;

  //position for placing
  //approach_place_1
  double approach_place_1_axis1_deg = 0;
  double approach_place_1_axis2_deg = 0;
  double approach_place_1_axis3_deg = 0;
  double approach_place_1_axis4_deg = 0;
  double approach_place_1_axis5_deg = 0;
  double approach_place_1_axis6_deg = 0;

  //place_1
  double place_1_axis1_deg = 0;
  double place_1_axis2_deg = 0;
  double place_1_axis3_deg = 0;
  double place_1_axis4_deg = 0;
  double place_1_axis5_deg = 0;
  double place_1_axis6_deg = 0;

  //approach_place_2
  double approach_place_2_axis1_deg = 0;
  double approach_place_2_axis2_deg = 0;
  double approach_place_2_axis3_deg = 0;
  double approach_place_2_axis4_deg = 0;
  double approach_place_2_axis5_deg = 0;
  double approach_place_2_axis6_deg = 0;

  //place_2
  double place_2_axis1_deg = 0;
  double place_2_axis2_deg = 0;
  double place_2_axis3_deg = 0;
  double place_2_axis4_deg = 0;
  double place_2_axis5_deg = 0;
  double place_2_axis6_deg = 0;

  //approach_place_3
  double approach_place_3_axis1_deg = 0;
  double approach_place_3_axis2_deg = 0;
  double approach_place_3_axis3_deg = 0;
  double approach_place_3_axis4_deg = 0;
  double approach_place_3_axis5_deg = 0;
  double approach_place_3_axis6_deg = 0;

  //place_3
  double place_3_axis1_deg = 0;
  double place_3_axis2_deg = 0;
  double place_3_axis3_deg = 0;
  double place_3_axis4_deg = 0;
  double place_3_axis5_deg = 0;
  double place_3_axis6_deg = 0;

  /*****convert the value from degree to radian*****/
  //look_table
  double look_table_axis1_rad = look_table_axis1_deg*M_PI/180;
  double look_table_axis2_rad = look_table_axis2_deg*M_PI/180;
  double look_table_axis3_rad = look_table_axis3_deg*M_PI/180;
  double look_table_axis4_rad = look_table_axis4_deg*M_PI/180;
  double look_table_axis5_rad = look_table_axis5_deg*M_PI/180;
  double look_table_axis6_rad = look_table_axis6_deg*M_PI/180;

  //approach_pick_1
  double approach_pick_1_axis1_rad = approach_pick_1_axis1_deg*M_PI/180;
  double approach_pick_1_axis2_rad = approach_pick_1_axis2_deg*M_PI/180;
  double approach_pick_1_axis3_rad = approach_pick_1_axis3_deg*M_PI/180;
  double approach_pick_1_axis4_rad = approach_pick_1_axis4_deg*M_PI/180;
  double approach_pick_1_axis5_rad = approach_pick_1_axis5_deg*M_PI/180;
  double approach_pick_1_axis6_rad = approach_pick_1_axis6_deg*M_PI/180;

  //pick_1
  double pick_1_axis1_rad = pick_1_axis1_deg*M_PI/180;
  double pick_1_axis2_rad = pick_1_axis2_deg*M_PI/180;
  double pick_1_axis3_rad = pick_1_axis3_deg*M_PI/180;
  double pick_1_axis4_rad = pick_1_axis4_deg*M_PI/180;
  double pick_1_axis5_rad = pick_1_axis5_deg*M_PI/180;
  double pick_1_axis6_rad = pick_1_axis6_deg*M_PI/180;

  //approach_pick_2
  double approach_pick_2_axis1_rad = approach_pick_2_axis1_deg*M_PI/180;
  double approach_pick_2_axis2_rad = approach_pick_2_axis2_deg*M_PI/180;
  double approach_pick_2_axis3_rad = approach_pick_2_axis3_deg*M_PI/180;
  double approach_pick_2_axis4_rad = approach_pick_2_axis4_deg*M_PI/180;
  double approach_pick_2_axis5_rad = approach_pick_2_axis5_deg*M_PI/180;
  double approach_pick_2_axis6_rad = approach_pick_2_axis6_deg*M_PI/180;

  //pick_2
  double pick_2_axis1_rad = pick_2_axis1_deg*M_PI/180;
  double pick_2_axis2_rad = pick_2_axis2_deg*M_PI/180;
  double pick_2_axis3_rad = pick_2_axis3_deg*M_PI/180;
  double pick_2_axis4_rad = pick_2_axis4_deg*M_PI/180;
  double pick_2_axis5_rad = pick_2_axis5_deg*M_PI/180;
  double pick_2_axis6_rad = pick_2_axis6_deg*M_PI/180;

  //approach_pick_3
  double approach_pick_3_axis1_rad = approach_pick_3_axis1_deg*M_PI/180;
  double approach_pick_3_axis2_rad = approach_pick_3_axis2_deg*M_PI/180;
  double approach_pick_3_axis3_rad = approach_pick_3_axis3_deg*M_PI/180;
  double approach_pick_3_axis4_rad = approach_pick_3_axis4_deg*M_PI/180;
  double approach_pick_3_axis5_rad = approach_pick_3_axis5_deg*M_PI/180;
  double approach_pick_3_axis6_rad = approach_pick_3_axis6_deg*M_PI/180;

  //pick_3
  double pick_3_axis1_rad = pick_3_axis1_deg*M_PI/180;
  double pick_3_axis2_rad = pick_3_axis2_deg*M_PI/180;
  double pick_3_axis3_rad = pick_3_axis3_deg*M_PI/180;
  double pick_3_axis4_rad = pick_3_axis4_deg*M_PI/180;
  double pick_3_axis5_rad = pick_3_axis5_deg*M_PI/180;
  double pick_3_axis6_rad = pick_3_axis6_deg*M_PI/180;

  //approach_place_1
  double approach_place_1_axis1_rad = approach_place_1_axis1_deg*M_PI/180;
  double approach_place_1_axis2_rad = approach_place_1_axis2_deg*M_PI/180;
  double approach_place_1_axis3_rad = approach_place_1_axis3_deg*M_PI/180;
  double approach_place_1_axis4_rad = approach_place_1_axis4_deg*M_PI/180;
  double approach_place_1_axis5_rad = approach_place_1_axis5_deg*M_PI/180;
  double approach_place_1_axis6_rad = approach_place_1_axis6_deg*M_PI/180;

  //place_1
  double place_1_axis1_rad = place_1_axis1_deg*M_PI/180;
  double place_1_axis2_rad = place_1_axis2_deg*M_PI/180;
  double place_1_axis3_rad = place_1_axis3_deg*M_PI/180;
  double place_1_axis4_rad = place_1_axis4_deg*M_PI/180;
  double place_1_axis5_rad = place_1_axis5_deg*M_PI/180;
  double place_1_axis6_rad = place_1_axis6_deg*M_PI/180;

  //approach_place_2
  double approach_place_2_axis1_rad = approach_place_2_axis1_deg*M_PI/180;
  double approach_place_2_axis2_rad = approach_place_2_axis2_deg*M_PI/180;
  double approach_place_2_axis3_rad = approach_place_2_axis3_deg*M_PI/180;
  double approach_place_2_axis4_rad = approach_place_2_axis4_deg*M_PI/180;
  double approach_place_2_axis5_rad = approach_place_2_axis5_deg*M_PI/180;
  double approach_place_2_axis6_rad = approach_place_2_axis6_deg*M_PI/180;

  //place_2
  double place_2_axis1_rad = place_2_axis1_deg*M_PI/180;
  double place_2_axis2_rad = place_2_axis2_deg*M_PI/180;
  double place_2_axis3_rad = place_2_axis3_deg*M_PI/180;
  double place_2_axis4_rad = place_2_axis4_deg*M_PI/180;
  double place_2_axis5_rad = place_2_axis5_deg*M_PI/180;
  double place_2_axis6_rad = place_2_axis6_deg*M_PI/180;

  //approach_place_3
  double approach_place_3_axis1_rad = approach_place_3_axis1_deg*M_PI/180;
  double approach_place_3_axis2_rad = approach_place_3_axis2_deg*M_PI/180;
  double approach_place_3_axis3_rad = approach_place_3_axis3_deg*M_PI/180;
  double approach_place_3_axis4_rad = approach_place_3_axis4_deg*M_PI/180;
  double approach_place_3_axis5_rad = approach_place_3_axis5_deg*M_PI/180;
  double approach_place_3_axis6_rad = approach_place_3_axis6_deg*M_PI/180;

  //place_3
  double place_3_axis1_rad = place_3_axis1_deg*M_PI/180;
  double place_3_axis2_rad = place_3_axis2_deg*M_PI/180;
  double place_3_axis3_rad = place_3_axis3_deg*M_PI/180;
  double place_3_axis4_rad = place_3_axis4_deg*M_PI/180;
  double place_3_axis5_rad = place_3_axis5_deg*M_PI/180;
  double place_3_axis6_rad = place_3_axis6_deg*M_PI/180;

  //insert the values to waypoints, do not change
  //in order of {joint1,joint2,joint3,joint4,joint5,joint6}, in radian
  std::vector<double> home = {0,0,0,0,0,0}; // do not change.
  std::vector<double> look_table = {look_table_axis1_rad,
                                    look_table_axis2_rad,
                                    look_table_axis3_rad,
                                    look_table_axis4_rad,
                                    look_table_axis5_rad,
                                    look_table_axis6_rad};

  std::vector<double> approach_place_1 = {approach_place_1_axis1_rad,
                                        approach_place_1_axis2_rad,
                                        approach_place_1_axis3_rad,
                                        approach_place_1_axis4_rad,
                                        approach_place_1_axis5_rad,
                                        approach_place_1_axis6_rad};
  std::vector<double> place_1 = {place_1_axis1_rad,
                               place_1_axis2_rad,
                               place_1_axis3_rad,
                               place_1_axis4_rad,
                               place_1_axis5_rad,
                               place_1_axis6_rad};

  std::vector<double> approach_place_2 = {approach_place_2_axis1_rad,
                                          approach_place_2_axis2_rad,
                                          approach_place_2_axis3_rad,
                                          approach_place_2_axis4_rad,
                                          approach_place_2_axis5_rad,
                                          approach_place_2_axis6_rad};
  std::vector<double> place_2 = {place_2_axis1_rad,
                                 place_2_axis2_rad,
                                 place_2_axis3_rad,
                                 place_2_axis4_rad,
                                 place_2_axis5_rad,
                                 place_2_axis6_rad};

  std::vector<double> approach_place_3 = {approach_place_3_axis1_rad,
                                          approach_place_3_axis2_rad,
                                          approach_place_3_axis3_rad,
                                          approach_place_3_axis4_rad,
                                          approach_place_3_axis5_rad,
                                          approach_place_3_axis6_rad};
  std::vector<double> place_3 = {place_3_axis1_rad,
                                 place_3_axis2_rad,
                                 place_3_axis3_rad,
                                 place_3_axis4_rad,
                                 place_3_axis5_rad,
                                 place_3_axis6_rad};

  //(1) waypoints to pick
  std::vector<double> approach_pick_1 = {approach_pick_1_axis1_rad,
                                         approach_pick_1_axis2_rad,
                                         approach_pick_1_axis3_rad,
                                         approach_pick_1_axis4_rad,
                                         approach_pick_1_axis5_rad,
                                         approach_pick_1_axis6_rad};
  std::vector<double> pick_1 = {pick_1_axis1_rad,
                                pick_1_axis2_rad,
                                pick_1_axis3_rad,
                                pick_1_axis4_rad,
                                pick_1_axis5_rad,
                                pick_1_axis6_rad};

  //(2) waypoints to pick
  std::vector<double> approach_pick_2 = {approach_pick_2_axis1_rad,
                                         approach_pick_2_axis2_rad,
                                         approach_pick_2_axis3_rad,
                                         approach_pick_2_axis4_rad,
                                         approach_pick_2_axis5_rad,
                                         approach_pick_2_axis6_rad};
  std::vector<double> pick_2 = {pick_2_axis1_rad,
                                pick_2_axis2_rad,
                                pick_2_axis3_rad,
                                pick_2_axis4_rad,
                                pick_2_axis5_rad,
                                pick_2_axis6_rad};

  //(3) waypoints to pick
  std::vector<double> approach_pick_3 = {approach_pick_3_axis1_rad,
                                         approach_pick_3_axis2_rad,
                                         approach_pick_3_axis3_rad,
                                         approach_pick_3_axis4_rad,
                                         approach_pick_3_axis5_rad,
                                         approach_pick_3_axis6_rad};
  std::vector<double> pick_3 = {pick_3_axis1_rad,
                                pick_3_axis2_rad,
                                pick_3_axis3_rad,
                                pick_3_axis4_rad,
                                pick_3_axis5_rad,
                                pick_3_axis6_rad};

  /*****open gripper befor start*****/
  if (goal_dynamixel_command_client_.call(open))
  {
    ROS_INFO ("open gripper");
  }
  else
  {
    ROS_ERROR("failed to open gripper");
    return 1;
  }
  sleep(0.5);

  /*****start from home position*****/
  arm.setJointValueTarget(home);
  arm.move();
  ROS_INFO("Arm is moving to home position");
  sleep(0.5);

  /*****(1) pick cylinder*****/
  arm.setJointValueTarget(look_table);
  arm.move();
  ROS_INFO("Arm is moving to look_table position");
  sleep(0.5);

  /*****release gripper*****/
  if (goal_dynamixel_command_client_.call(release))
  {
    ROS_INFO ("release gripper");
  }
  else
  {
    ROS_ERROR("failed to release gripper");
    return 1;
  }
  sleep(0.5);

  arm.setJointValueTarget(approach_pick_1);
  arm.move();
  ROS_INFO("Arm is moving to approach_pick position");
  sleep(0.5);

  arm.setJointValueTarget(pick_1);
  arm.move();
  ROS_INFO("Arm is moving to pick position");
  sleep(0.5);

  /*****close gripper*****/
  if (goal_dynamixel_command_client_.call(close))
  {
    ROS_INFO ("close gripper");
  }
  else
  {
    ROS_ERROR("failed to close gripper");
    return 1;
  }
  sleep(0.5);

  arm.setJointValueTarget(approach_pick_1);
  arm.move();
  ROS_INFO("Arm is moving to approach_pick position");
  sleep(0.5);

  arm.setJointValueTarget(look_table);
  arm.move();
  ROS_INFO("Arm is moving to look_table position");
  sleep(0.5);

  /*****(1) place cylinder*****/
  arm.setJointValueTarget(approach_place_1);
  arm.move();
  ROS_INFO("Arm is moving to approach_place position");
  sleep(0.5);

  arm.setJointValueTarget(place_1);
  arm.move();
  ROS_INFO("Arm is moving to place position");
  sleep(0.5);

  /*****release gripper*****/
  if (goal_dynamixel_command_client_.call(release))
  {
    ROS_INFO ("release gripper");
  }
  else
  {
    ROS_ERROR("failed to release gripper");
    return 1;
  }
  sleep(0.5);

  arm.setJointValueTarget(approach_place_1);
  arm.move();
  ROS_INFO("Arm is moving to approach_place position");
  sleep(0.5);

  /*****(1) complete pick and place cylinder*****/

  /*****(2) pick triangular prism*****/
  arm.setJointValueTarget(look_table);
  arm.move();
  ROS_INFO("Arm is moving to look_table position");
  sleep(0.5);

  if (goal_dynamixel_command_client_.call(release))
  {
    ROS_INFO ("open gripper");
  }
  else
  {
    ROS_ERROR("failed to open gripper");
    return 1;
  }
  sleep(0.5);

  arm.setJointValueTarget(approach_pick_2);
  arm.move();
  ROS_INFO("Arm is moving to approach_pick position");
  sleep(0.5);

  arm.setJointValueTarget(pick_2);
  arm.move();
  ROS_INFO("Arm is moving to pick position");
  sleep(0.5);

  /*****close gripper*****/
  if (goal_dynamixel_command_client_.call(close))
  {
    ROS_INFO ("close gripper");
  }
  else
  {
    ROS_ERROR("failed to close gripper");
    return 1;
  }
  sleep(0.5);

  arm.setJointValueTarget(approach_pick_2);
  arm.move();
  ROS_INFO("Arm is moving to approach_pick position");
  sleep(0.5);

  arm.setJointValueTarget(look_table);
  arm.move();
  ROS_INFO("Arm is moving to look_table position");
  sleep(0.5);

  /*****(2) place triangular prism*****/
  arm.setJointValueTarget(approach_place_2);
  arm.move();
  ROS_INFO("Arm is moving to approach_place position");
  sleep(0.5);

  arm.setJointValueTarget(place_2);
  arm.move();
  ROS_INFO("Arm is moving to place position");
  sleep(0.5);

  /*****release gripper*****/
  if (goal_dynamixel_command_client_.call(release))
  {
    ROS_INFO ("release gripper");
  }
  else
  {
    ROS_ERROR("failed to release gripper");
    return 1;
  }
  sleep(0.5);

  arm.setJointValueTarget(approach_place_2);
  arm.move();
  ROS_INFO("Arm is moving to approach_place position");
  sleep(0.5);
  /*****(2) complete pick and place triangular prism*****/

  /*****(3) pick cylinder*****/
  arm.setJointValueTarget(look_table);
  arm.move();
  ROS_INFO("Arm is moving to look_table position");
  sleep(0.5);

  /*****release gripper*****/
  if (goal_dynamixel_command_client_.call(release))
  {
    ROS_INFO ("release gripper");
  }
  else
  {
    ROS_ERROR("failed to release gripper");
    return 1;
  }
  sleep(0.5);

  arm.setJointValueTarget(approach_pick_3);
  arm.move();
  ROS_INFO("Arm is moving to approach_pick position");
  sleep(0.5);

  arm.setJointValueTarget(pick_3);
  arm.move();
  ROS_INFO("Arm is moving to pick position");
  sleep(0.5);

  /*****close gripper*****/
  if (goal_dynamixel_command_client_.call(close_mid))
  {
    ROS_INFO ("close gripper");
  }
  else
  {
    ROS_ERROR("failed to close gripper");
    return 1;
  }
  sleep(0.5);

  arm.setJointValueTarget(approach_pick_3);
  arm.move();
  ROS_INFO("Arm is moving to approach_pick position");
  sleep(0.5);

  arm.setJointValueTarget(look_table);
  arm.move();
  ROS_INFO("Arm is moving to look_table position");
  sleep(0.5);

  /*****(1) place cylinder*****/
  arm.setJointValueTarget(approach_place_3);
  arm.move();
  ROS_INFO("Arm is moving to approach_place position");
  sleep(0.5);

  arm.setJointValueTarget(place_3);
  arm.move();
  ROS_INFO("Arm is moving to place position");
  sleep(0.5);

  /*****release gripper*****/
  if (goal_dynamixel_command_client_.call(release))
  {
    ROS_INFO ("release gripper");
  }
  else
  {
    ROS_ERROR("failed to release gripper");
    return 1;
  }
  sleep(0.5);

  arm.setJointValueTarget(approach_place_3);
  arm.move();
  ROS_INFO("Arm is moving to approach_place position");
  sleep(0.5);

  /*****go back to home position*****/
  arm.setJointValueTarget(home);
  arm.move();
  ROS_INFO("Arm is moving to home position");
  sleep(0.5);

  ros::shutdown();
  return 0;
}

/***** original positions for placing *****/
//approach_place_1
// double approach_place_1_axis1_deg = 138;
// double approach_place_1_axis2_deg = -7;
// double approach_place_1_axis3_deg = 115;
// double approach_place_1_axis4_deg = 39;
// double approach_place_1_axis5_deg = 90;
// double approach_place_1_axis6_deg = 30;
//
// //place_1
// double place_1_axis1_deg = 138;
// double place_1_axis2_deg = -17;
// double place_1_axis3_deg = 122;
// double place_1_axis4_deg = 55;
// double place_1_axis5_deg = 90;
// double place_1_axis6_deg = 27;
//
// //approach_place_2
// double approach_place_2_axis1_deg = 116;
// double approach_place_2_axis2_deg = 2;
// double approach_place_2_axis3_deg = 118;
// double approach_place_2_axis4_deg = 29;
// double approach_place_2_axis5_deg = 90;
// double approach_place_2_axis6_deg = 21;
//
// //place_2
// double place_2_axis1_deg = 116;
// double place_2_axis2_deg = -12;
// double place_2_axis3_deg = 126;
// double place_2_axis4_deg = 50;
// double place_2_axis5_deg = 90;
// double place_2_axis6_deg = 21;
//
// //approach_place_3
// double approach_place_3_axis1_deg = 88;
// double approach_place_3_axis2_deg = -5;
// double approach_place_3_axis3_deg = 122;
// double approach_place_3_axis4_deg = 29;
// double approach_place_3_axis5_deg = 90;
// double approach_place_3_axis6_deg = 7;
//
// //place_3 // need modify
// double place_3_axis1_deg = 88;
// double place_3_axis2_deg = -16;
// double place_3_axis3_deg = 119;
// double place_3_axis4_deg = 47;
// double place_3_axis5_deg = 90;
// double place_3_axis6_deg = 7;
