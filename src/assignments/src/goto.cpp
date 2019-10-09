#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

//Goal Pose
float goalX = 5;
float goalY = 5;
float goalTHETA = 0;	

//Robot Pose
float robot_pose_x;								
float robot_pose_y;								
float quat_x, quat_y, quat_z, quat_w;		
double robot_orientation;									

						
//flags to move
bool rot = true;
bool move = false;
bool rot2 = false;

//laser range
float laser1;
float laser2;


//Function to return the distance between 2 points
float distance(float x1, float y1, float x2, float y2) {
	float vect_x = x2 - x1;
	float vect_y = y2 - y1;
	float sum_square = (((vect_x) * (vect_x)) + ((vect_y) * (vect_y)));
	float dist = sqrt(sum_square);
	return dist;
}


//Odometry message Callback function
void pose_groundMessageReceived(const nav_msgs::Odometry&msg) {		
	robot_pose_x = msg.pose.pose.position.x;				//Extracting Current Position (x,y) of Robot
	robot_pose_y = msg.pose.pose.position.y;

	quat_x = msg.pose.pose.orientation.x;				//Extracting current Quaternion Values of Robot
	quat_y = msg.pose.pose.orientation.y;
	quat_z = msg.pose.pose.orientation.z;
	quat_w = msg.pose.pose.orientation.w;

	tf::Quaternion q(quat_x, quat_y, quat_z, quat_w);		//Create Quaternion to extract robot_orientation angle

	robot_orientation = tf::getYaw(q);			//Define robot_orientation angle between positive x-axis and target point

}

void pose_laserMessageReceived(const sensor_msgs::LaserScan& msg){
	
	 laser1 = msg.ranges[360];
	 laser2 = msg.ranges[460];
	
}


int main(int argc,char ** argv) {
	ros::init(argc,argv,"gotoposition");
	ros::NodeHandle nh_;
	
	ros::Subscriber pose_global = nh_.subscribe("/odometry/filtered",1000,&pose_groundMessageReceived);
	ros::Subscriber laser = nh_.subscribe("/scan", 1000 , &pose_laserMessageReceived);
	ros::Publisher pubVel = nh_.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel",1000);

	//Twist velocity variable
	geometry_msgs::Twist vel;

	ros::spinOnce();
	//Calculate vector to goal
	float vect_x = abs(goalX) - abs(robot_pose_x);				
	float vect_y = abs(goalY) - abs(robot_pose_y);

	int sign=vect_x/abs(vect_x);
	vect_x=sign*vect_x;
	sign=vect_y/abs(vect_y);
	vect_y=sign*vect_y;

	float angle_diff = atan2(vect_y,vect_x);	//atan2(y,x) returns the angle (-pi and pi) of the x-axis and the Goal Point(goalX,goalY) 
	float goal_dist = distance(goalX, goalY, robot_pose_x, robot_pose_y);						//Distance from Goal Point and Origin (WORLD FRAME)
	float init_robot_x=	robot_pose_x;
	float init_robot_y=	robot_pose_y;

	ros::Rate rate(3);

	while (ros::ok()) {
		ros::spinOnce();

	
		float dist_remainder = distance(init_robot_x, init_robot_y, robot_pose_x, robot_pose_y);		//Distance from Origin (WORLD FRAME) and Robot Ground Pose Coordinates

		//Rotate the robot to goal robot_orientation orientation 
		if(rot == true) {
			move = false;
			rot = true;
			//ros::spinOnce();
			vel.linear.x = 0;
			pubVel.publish(vel);
			vel.angular.z = 0.5;		//While rot is true, rotate robot counterclockwise
			pubVel.publish(vel);
			
			std::cout<<"First Alignment. Current robot's angle: " << robot_orientation << " vs goal's angle: " << angle_diff<<std::endl;

			if(((robot_orientation + .1) >= angle_diff) && ((robot_orientation - .1) <= angle_diff)) {			//When robot is facing target orientation, stop rotating
				rot = true;
				move = false;
				vel.angular.x = 0;
				vel.angular.z = 0;
				pubVel.publish(vel);
				std::cout<<"STOP"<<std::endl;

			}
		}

		if(laser1 < dist_remainder){

			move = false;
			vel.linear.x = 0;
			pubVel.publish(vel);
			ros::spinOnce();

			while(laser2 < 15 && ros::ok()){
				std::cout << "test2" << std::endl;
				ros::spinOnce();
				rot = true;
				vel.angular.z = 0.5;
				pubVel.publish(vel);


			}
			std::cout << "TEST3" << std::endl;
			//ros::spinOnce();
			move = false;
			rot = false;
			vel.angular.x = 0;
			pubVel.publish(vel);
			vel.angular.z = 0;
			pubVel.publish(vel);

			float init_x2 = init_robot_x + 2;
			float init_y2 = init_robot_y + 2;
			
			while(laser1 > 7.2 && ros::ok()){


			std::cout << "test4" << std::endl;


				ros::spinOnce();
				move = true;
				rot = false;
				vel.linear.x = 0.5;
				pubVel.publish(vel);


			}
						std::cout << "TEST5" << std::endl;


			move = false;
			rot = true;
			vel.linear.x = 0;
			pubVel.publish(vel);
			vel.linear.z = 0;
			pubVel.publish(vel);


		}

		else{


			move = true;
			rot = false;
			vel.linear.x = 0.5;
			pubVel.publish(vel);

			if(dist_remainder >= goal_dist) {			//When robot reaches target distance, stop moving
				//ros::spinOnce();
				move = false;
				rot = false;
				vel.linear.x = 0;
				pubVel.publish(vel);
				vel.angular.z = 0;
				pubVel.publish(vel);
				std::cout<<"STOP"<<std::endl;

			}
		
			

	}
	
	rate.sleep();

}

}