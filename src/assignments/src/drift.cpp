#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Float64.h>

geometry_msgs::Point position;
geometry_msgs::Point coor;

void linearMSG(const nav_msgs::Odometry& msg){

	position = msg.pose.pose.position;	

}

void angularMSG(const nav_msgs::Odometry& msg){

	coor = msg.pose.pose.position;

}


int main(int argc, char **argv){

ros::init(argc, argv, "drift");

ros::NodeHandle nh;

ros::Subscriber sublinear = nh.subscribe("/base_pose_ground_truth",1000, &linearMSG);

ros::Subscriber subangular = nh.subscribe("/pioneer/odom",1000, &angularMSG);

ros::Publisher drift = nh.advertise<std_msgs::Float64>("/posedrift",1000);

ros::Rate rate(10);

std_msgs::Float64 pose;


while(ros::ok()){

ros::spinOnce();

pose.data = sqrt(((position.x - coor.x)*(position.x - coor.x))+((position.y - coor.y)*(position.y - coor.y)));

drift.publish(pose);

rate.sleep();

}







}
