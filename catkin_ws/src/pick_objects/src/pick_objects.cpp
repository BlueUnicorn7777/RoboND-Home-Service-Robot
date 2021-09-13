#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//x,y,orientation


void send_goal(MoveBaseClient & ac, double *pos){
 move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pos[0];
  goal.target_pose.pose.position.y = pos[1];
  goal.target_pose.pose.orientation.w = pos[2];

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached Goal ----");
  else
    ROS_INFO("Failed to reach goal ----");
}

int main(int argc, char** argv){

   double pickUpGoal[3]  = {2.90, 3.7,0.4};
   double dropOffGoal[3] = {-0.09, -4.31,0.4};
   double home[3]={0.2,-0.20,0.4};


  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  send_goal(ac,pickUpGoal);
  sleep(5);
  send_goal(ac,dropOffGoal);
  sleep(5);
  send_goal(ac,home);
  ROS_INFO("Finished Task !! :-)");


  return 0;
}


