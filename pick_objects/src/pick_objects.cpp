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

  move_base_msgs::MoveBaseGoal goal;
  //move_base_msgs::MoveBaseGoal goal_two;
  
  // ***goal one setup***
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -2.0;
  goal.target_pose.pose.position.y = -4.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("The robot is traveling to the pick up zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The robot has reached the pick up zone");
    
    // Wait 5 seconds to simulate object pick up
    ros::Duration(5.0).sleep();
    ROS_INFO("The robot is picking up the virtual object");
    
    // ***goal two setup***
  	// set up the frame parameters
  	//goal_two.target_pose.header.frame_id = "map";
  	//goal_two.target_pose.header.stamp = ros::Time::now();

  	// Define a position and orientation for the robot to reach
  	goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 3.0;
  	goal.target_pose.pose.orientation.w = 1.0;
    
    // Send the goal position and orientation for the robot to reach
  	ROS_INFO("The robot is traveling to the drop off zone");
  	ac.sendGoal(goal);

  	// Wait an infinite time for the results
  	ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
    	ROS_INFO("The robot has reached the drop off zone");
    	ROS_INFO("The robot is dropping off the virtual object");
    }
    else
    	ROS_INFO("The robot FAILED to reach the drop off zone");
    
  }
  else
    ROS_INFO("The robot FAILED to reach the pick up zone");

  return 0;
}