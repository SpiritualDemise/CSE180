#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


int main(int argc, char** argv){


ros::init(argc, argv, "moverobot");
ros::NodeHandle nh;

ros::Publisher vel = nh.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 1000);
ros::Publisher ang = nh.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 1000);

ros::Rate rate(10);

geometry_msgs::Twist linear;
geometry_msgs::Twist angular;

linear.linear.x = 1;
angular.angular.z = M_PI/2;


while(ros::ok()){


ang.publish(angular);
ros::Duration(1).sleep();

vel.publish(linear);
ros::Duration(1).sleep();

}

}
