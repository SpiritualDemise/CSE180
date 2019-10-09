#include<ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  int main(int argc, char** argv){

    ros::init(argc, argv, "simple_navigation_goals");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true); 
  
  //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){

        ROS_INFO("Waiting for the move_base action server to come up");
    
    } 
  
  move_base_msgs::MoveBaseGoal goal;
  
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
 
  int i = 0;
  int j = 1;
  int totalpoints = 7;

  float values[] = {0 ,4 , 4 , 0 , 9 , 0 , 9 , 3 , 4 , 3 , 4 , 6 ,9 , 6};

  goal.target_pose.pose.position.x = values[i];
  goal.target_pose.pose.position.y = values[j];
  goal.target_pose.pose.orientation.w = 1;

  ROS_INFO("Sending goal");


  ROS_INFO_STREAM("X-Coordinate: " << goal.target_pose.pose.position.x);
  ROS_INFO_STREAM("Y-Coordinate: " << goal.target_pose.pose.position.y);


  ac.sendGoal(goal);
  
  totalpoints--;
  
  ac.waitForResult();
 
while(ros::ok() && totalpoints != 0){

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        
        ROS_INFO(" Point Reached! ");

        i = i + 2;
        j = j + 2;


        goal.target_pose.pose.position.x = values[i];
        goal.target_pose.pose.position.y = values[j];


        ROS_INFO_STREAM("X-Coordinate: " << goal.target_pose.pose.position.x);
        ROS_INFO_STREAM("Y-Coordinate: " << goal.target_pose.pose.position.y);

        ac.sendGoal(goal);

        totalpoints--;
        ac.waitForResult();

    }
    else{

  ROS_INFO(" OOF OOF OOF OOF ");
  
    }

}
  
  return 0;
  
}