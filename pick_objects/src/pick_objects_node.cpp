#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  // goal pose: (x, y), (w, x, y, z,)  
  float pickUpPose[6] = {7.5,-2.85, 0.707, 0.0 , 0.0, 0.707}; // rotation to 90 deg (1.57 rad)
  float dropOffPose[6] = {0.0, 7.6, 0.0, 0.0, 0.0, 1.0}; // rotation to 180 deg (3.14 rad) 

  move_base_msgs::MoveBaseGoal pickUpZone;
  move_base_msgs::MoveBaseGoal dropOffZone;

  // set up the frame parameters
  pickUpZone.target_pose.header.frame_id = "map";
  pickUpZone.target_pose.header.stamp = ros::Time::now();

  dropOffZone.target_pose.header.frame_id = "map";
  dropOffZone.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to pick the object
  pickUpZone.target_pose.pose.position.x = pickUpPose[0];
  pickUpZone.target_pose.pose.position.y = pickUpPose[1];
  pickUpZone.target_pose.pose.orientation.w = pickUpPose[2];
  pickUpZone.target_pose.pose.orientation.x = pickUpPose[3];
  pickUpZone.target_pose.pose.orientation.y = pickUpPose[4];
  pickUpZone.target_pose.pose.orientation.z = pickUpPose[5];

  // Define a position and orientation for the robot to drop off the object
  dropOffZone.target_pose.pose.position.x = dropOffPose[0];
  dropOffZone.target_pose.pose.position.y = dropOffPose[1];
  dropOffZone.target_pose.pose.orientation.w = dropOffPose[2];
  dropOffZone.target_pose.pose.orientation.x = dropOffPose[3];
  dropOffZone.target_pose.pose.orientation.y = dropOffPose[4];
  dropOffZone.target_pose.pose.orientation.z = dropOffPose[5];

  // Send the object pick up position and orientation for the robot
  ROS_INFO("The robot is moving to the object pickup zone");
  ac.sendGoal(pickUpZone);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    
    ROS_INFO("The robot riched the object pick up zone");
    // Wait 5 sec for move_base action server to come up
    ros::Duration(5.0).sleep();
    ROS_INFO("The robot successfully pick up the object");
  }
  else
    ROS_INFO("The robot failed to rich the object pickup zone");

  // Send the object drop off position and orientation for the robot
  ROS_INFO("The robot is moving to the object drop off zone");
  ac.sendGoal( dropOffZone);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("The robot riched the object dropped off zone");
    // Wait 5 sec for move_base action server to come up
    ros::Duration(5.0).sleep();
    ROS_INFO("The robot successfully dropped off the object");
  }
  else
    ROS_INFO("The robot failed to rich the object drop off zone");

  return 0;
}