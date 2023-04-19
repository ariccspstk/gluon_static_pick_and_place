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

/* Convert degree to radian function
 * ================================= */
double convertDegtoRad(double degree)
{
  double radian = degree*M_PI/180;
  return radian;
}

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

  // Set the position of gripper, values refer to DynamixelWizard 2.0 --> goal position
  // Set gripper to open position
  dynamixel_workbench_msgs::DynamixelCommand open;
  open.request.id_1 = DXL_LEFT;
  open.request.id_2 = DXL_RIGHT;
  open.request.addr_name_1 = ADDRESS;
  open.request.addr_name_2 = ADDRESS;
  open.request.value_1 = 330;
  open.request.value_2 = 730;

  // Set gripper to release position
  dynamixel_workbench_msgs::DynamixelCommand release;
  release.request.id_1 = DXL_LEFT;
  release.request.id_2 = DXL_RIGHT;
  release.request.addr_name_1 = ADDRESS;
  release.request.addr_name_2 = ADDRESS;
  release.request.value_1 = 470;
  release.request.value_2 = 550;

  // Set gripper to close position
  dynamixel_workbench_msgs::DynamixelCommand close_mid;
  close_mid.request.id_1 = DXL_LEFT;
  close_mid.request.id_2 = DXL_RIGHT;
  close_mid.request.addr_name_1 = ADDRESS;
  close_mid.request.addr_name_2 = ADDRESS;
  close_mid.request.value_1 = 520;
  close_mid.request.value_2 = 500;

  // Set gripper close position
  dynamixel_workbench_msgs::DynamixelCommand close;
  close.request.id_1 = DXL_LEFT;
  close.request.id_2 = DXL_RIGHT;
  close.request.addr_name_1 = ADDRESS;
  close.request.addr_name_2 = ADDRESS;
  close.request.value_1 = 542;
  close.request.value_2 = 482;

  /* ==================
   * Set Waypoints Pick
   * ================== */

  // Picking & Placing position starts from left -> right
  // Add in your desired inputs in degrees into convertDegtoRad() function

  // Look table position
  double look_table_axis1 = convertDegtoRad(-89);
  double look_table_axis2 = convertDegtoRad(-21);
  double look_table_axis3 = convertDegtoRad(74);
  double look_table_axis4 = convertDegtoRad(97);
  double look_table_axis5 = convertDegtoRad(90);
  double look_table_axis6 = convertDegtoRad(-3);

  // Approach pick 1 position
  double approach_pick_1_axis1 = convertDegtoRad(-112);
  double approach_pick_1_axis2 = convertDegtoRad(-78);
  double approach_pick_1_axis3 = convertDegtoRad(48);
  double approach_pick_1_axis4 = convertDegtoRad(39);
  double approach_pick_1_axis5 = convertDegtoRad(88);
  double approach_pick_1_axis6 = convertDegtoRad(-28);

  // Pick 1 position
  double pick_1_axis1 = convertDegtoRad(-112);
  double pick_1_axis2 = convertDegtoRad(-84);
  double pick_1_axis3 = convertDegtoRad(43);
  double pick_1_axis4 = convertDegtoRad(40);
  double pick_1_axis5 = convertDegtoRad(88);
  double pick_1_axis6 = convertDegtoRad(-28);

  //(2) positions for triangular prism
  //approach_pick_2
  double approach_pick_2_axis1 = convertDegtoRad(-74);
  double approach_pick_2_axis2 = convertDegtoRad(-63);
  double approach_pick_2_axis3 = convertDegtoRad(96);
  double approach_pick_2_axis4 = convertDegtoRad(70);
  double approach_pick_2_axis5 = convertDegtoRad(86);
  double approach_pick_2_axis6 = convertDegtoRad(15);

  //pick_2
  double pick_2_axis1 = convertDegtoRad(-74);
  double pick_2_axis2 = convertDegtoRad(-68);
  double pick_2_axis3 = convertDegtoRad(93);
  double pick_2_axis4 = convertDegtoRad(72);
  double pick_2_axis5 = convertDegtoRad(86);
  double pick_2_axis6 = convertDegtoRad(15);

  //(3) positions for triangular prism
  //approach_pick_3
  double approach_pick_3_axis1 = convertDegtoRad(-48);
  double approach_pick_3_axis2 = convertDegtoRad(-72);
  double approach_pick_3_axis3 = convertDegtoRad(71);
  double approach_pick_3_axis4 = convertDegtoRad(53);
  double approach_pick_3_axis5 = convertDegtoRad(92);
  double approach_pick_3_axis6 = convertDegtoRad(40);

  //pick_3
  double pick_3_axis1 = convertDegtoRad(-48);
  double pick_3_axis2 = convertDegtoRad(-76);
  double pick_3_axis3 = convertDegtoRad(68);
  double pick_3_axis4 = convertDegtoRad(54);
  double pick_3_axis5 = convertDegtoRad(92);
  double pick_3_axis6 = convertDegtoRad(40);

  //position for placing
  //approach_place_1
  double approach_place_1_axis1 = convertDegtoRad(-88);
  double approach_place_1_axis2 = convertDegtoRad(-2);
  double approach_place_1_axis3 = convertDegtoRad(-115);
  double approach_place_1_axis4 = convertDegtoRad(-24);
  double approach_place_1_axis5 = convertDegtoRad(-90);
  double approach_place_1_axis6 = convertDegtoRad(158);

  //place_1
  double place_1_axis1 = convertDegtoRad(-86);
  double place_1_axis2 = convertDegtoRad(7);
  double place_1_axis3 = convertDegtoRad(-113);
  double place_1_axis4 = convertDegtoRad(-26);
  double place_1_axis5 = convertDegtoRad(-92);
  double place_1_axis6 = convertDegtoRad(160);

  //approach_place_2
  double approach_place_2_axis1 = convertDegtoRad(-117);
  double approach_place_2_axis2 = convertDegtoRad(-5);
  double approach_place_2_axis3 = convertDegtoRad(-114);
  double approach_place_2_axis4 = convertDegtoRad(-16);
  double approach_place_2_axis5 = convertDegtoRad(-91);
  double approach_place_2_axis6 = convertDegtoRad(150);

  //place_2
  double place_2_axis1 = convertDegtoRad(-117);
  double place_2_axis2 = convertDegtoRad(-1);
  double place_2_axis3 = convertDegtoRad(-121);
  double place_2_axis4 = convertDegtoRad(-27);
  double place_2_axis5 = convertDegtoRad(-91);
  double place_2_axis6 = convertDegtoRad(149);

  //approach_place_3
  double approach_place_3_axis1 = convertDegtoRad(-146);
  double approach_place_3_axis2 = convertDegtoRad(-1);
  double approach_place_3_axis3 = convertDegtoRad(-118);
  double approach_place_3_axis4 = convertDegtoRad(-28);
  double approach_place_3_axis5 = convertDegtoRad(-90);
  double approach_place_3_axis6 = convertDegtoRad(143);

  //place_3
  double place_3_axis1 = convertDegtoRad(-146);
  double place_3_axis2 = convertDegtoRad(2);
  double place_3_axis3 = convertDegtoRad(-120);
  double place_3_axis4 = convertDegtoRad(-33);
  double place_3_axis5 = convertDegtoRad(-90);
  double place_3_axis6 = convertDegtoRad(143);

  /* Insert the values to waypoints, *DO NOT CHANGE*.
     In order of {joint1,joint2,joint3,joint4,joint5,joint6}, in radians. */

  // Waypoint to home
  std::vector<double> home = {0,0,0,0,0,0}; // Default values for home postion *DO NOT CHANGE*.

  // Waypoint to look table
  std::vector<double> look_table = { look_table_axis1,
                                      look_table_axis2,
                                        look_table_axis3,
                                          look_table_axis4,
                                            look_table_axis5,
                                              look_table_axis6 };

  // Waypoint (1) to place
  std::vector<double> approach_place_1 = { approach_place_1_axis1,
                                            approach_place_1_axis2,
                                              approach_place_1_axis3,
                                                approach_place_1_axis4,
                                                  approach_place_1_axis5,
                                                    approach_place_1_axis6 };
  
  std::vector<double> place_1 = { place_1_axis1,
                                    place_1_axis2,
                                      place_1_axis3,
                                        place_1_axis4,
                                          place_1_axis5,
                                            place_1_axis6 };

  // Waypoint (2) to place
  std::vector<double> approach_place_2 = { approach_place_2_axis1,
                                            approach_place_2_axis2,
                                              approach_place_2_axis3,
                                                approach_place_2_axis4,
                                                  approach_place_2_axis5,
                                                    approach_place_2_axis6 };

  std::vector<double> place_2 = { place_2_axis1,
                                    place_2_axis2,
                                      place_2_axis3,
                                        place_2_axis4,
                                          place_2_axis5,
                                            place_2_axis6 };

  // Waypoint (3) to place
  std::vector<double> approach_place_3 = { approach_place_3_axis1,
                                            approach_place_3_axis2,
                                              approach_place_3_axis3,
                                                approach_place_3_axis4,
                                                  approach_place_3_axis5,
                                                    approach_place_3_axis6 };

  std::vector<double> place_3 = { place_3_axis1,
                                    place_3_axis2,
                                      place_3_axis3,
                                        place_3_axis4,
                                          place_3_axis5,
                                            place_3_axis6 };

  // Waypoint (1) to pick
  std::vector<double> approach_pick_1 = { approach_pick_1_axis1,
                                            approach_pick_1_axis2,
                                              approach_pick_1_axis3,
                                                approach_pick_1_axis4,
                                                  approach_pick_1_axis5,
                                                    approach_pick_1_axis6 };
                                          
  std::vector<double> pick_1 = {  pick_1_axis1,
                                    pick_1_axis2,
                                      pick_1_axis3,
                                        pick_1_axis4,
                                          pick_1_axis5,
                                           pick_1_axis6 };

  // Waypoint (2) to pick
  std::vector<double> approach_pick_2 = { approach_pick_2_axis1,
                                            approach_pick_2_axis2,
                                              approach_pick_2_axis3,
                                                approach_pick_2_axis4,
                                                  approach_pick_2_axis5,
                                                    approach_pick_2_axis6 };

  std::vector<double> pick_2 = {  pick_2_axis1,
                                    pick_2_axis2,
                                      pick_2_axis3,
                                        pick_2_axis4,
                                          pick_2_axis5,
                                            pick_2_axis6 };

  // Waypoint (3) to pick
  std::vector<double> approach_pick_3 = { approach_pick_3_axis1,
                                            approach_pick_3_axis2,
                                              approach_pick_3_axis3,
                                                approach_pick_3_axis4,
                                                  approach_pick_3_axis5,
                                                    approach_pick_3_axis6 };

  std::vector<double> pick_3 = {  pick_3_axis1,
                                    pick_3_axis2,
                                      pick_3_axis3,
                                        pick_3_axis4,
                                          pick_3_axis5,
                                            pick_3_axis6 };

  /* ========================
     Open gripper to begin
     ======================== */
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

  if (goal_dynamixel_command_client_.call(open))
  {
    ROS_INFO ("release gripper");
  }
  else
  {
    ROS_ERROR("failed to release gripper");
    return 1;
  }
  sleep(0.5);

  /*****go back to home position*****/
  arm.setJointValueTarget(home);
  arm.move();
  ROS_INFO("Arm is moving to home position");
  sleep(0.5);

  ros::shutdown();
  return 0;
}
