#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";

  float goals[2][2] = {{1.0,1.0},{3.0,1.2}};

   // Send the goal position and orientation for the robot to reach
  for (int i = 0; i < 2; i++)
  {
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = goals[i][0];
    goal.target_pose.pose.orientation.w = goals[i][1];

    ROS_INFO("Sending goal of position %f and orientation %f", goals[i][0], goals[i][1]);
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward to the first goal");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");

    ros::Time begin = ros::Time::now();

    ros::Time future = begin + ros::Duration(5.0);

    while(ros::Time::now() < future && i == 0)
    {
       ROS_INFO("Pause 5 seconds after reaching first pick up zone");
    }
  }

  return 0;
}