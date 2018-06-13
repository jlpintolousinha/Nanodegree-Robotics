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

  bool first_goal=false;
  bool second_goal=false;

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the pickup frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to pickup
  goal.target_pose.pose.position.x = 4.5;
  goal.target_pose.pose.position.y = 2.0;
  goal.target_pose.pose.orientation.w = 0.5;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending first goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && first_goal==false){
    ROS_INFO("Hooray, the base moved to the pickup zone");
    ROS_INFO("First goal reached!");
    first_goal = true;
  } 
  else
    ROS_INFO("The base failed to move for some reason");
    
  if (first_goal == true && second_goal == false){
  	sleep(5);
  	ROS_INFO("Sending second goal");
  	
  	move_base_msgs::MoveBaseGoal Next_goal;
  	Next_goal.target_pose.header.frame_id = "map";
 		Next_goal.target_pose.header.stamp = ros::Time::now();
  	
		// Define a position and orientation for the robot to  drop off
		Next_goal.target_pose.pose.position.x = -4.5;
		Next_goal.target_pose.pose.position.y = 1.0;
		Next_goal.target_pose.pose.orientation.w = -0.3;
    
    ac.sendGoal(Next_goal);
	 	ac.waitForResult();
	 	
		if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		  	ROS_INFO("Hooray, the base moved to the dropoff zone");
		  	ROS_INFO("Second goal reached!");
	 		  first_goal = false;
	 		  second_goal = true;
	 		  sleep(5);
	 		  exit(EXIT_FAILURE);
		}
		else
		  ROS_INFO("The base failed to move for some reason");
	}

  return 0;	
}
