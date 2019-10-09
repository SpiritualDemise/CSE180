#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <std_msgs/Bool.h>
#include <iostream>

//Global variables for data including laser scan, and odometry
nav_msgs::Odometry curr_robot_pose;
sensor_msgs::LaserScan laser;
geometry_msgs::Vector3 goal_pose;
bool startmoving;
std_msgs::Bool arrivedGoal;
//Thresholds for the goal position, orientation, and movement speed
//Can set these parameters as desired
const float THRESHOLD_ANGLE = 0.1, THRESHOLD_GOAL = 1.5, move_speed = 0.5;

void odomCB(const nav_msgs::Odometry::ConstPtr &msg) {
    curr_robot_pose = *msg;
    return;
}

void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laser = *msg;
    return;
}

void recievePoints(const geometry_msgs::Vector3 &msg){
    goal_pose.x = msg.x;
    goal_pose.y = msg.y;
}


void goCB(const std_msgs::Bool &msg){
    startmoving = msg.data;
}

//Calculate distance between two points
float distance2D(nav_msgs::Odometry pose1, geometry_msgs::Vector3 pose2) {
    float x = pose2.x - pose1.pose.pose.position.x;
    float y = pose2.y - pose1.pose.pose.position.y;
    return std::hypotf(x, y);
}

//Calculate the desired angle
float calculateDesiredAngle(nav_msgs::Odometry rob_pose, geometry_msgs::Vector3 goal){
    float vec_x = goal.x - rob_pose.pose.pose.position.x;
    float vec_y = goal.y - rob_pose.pose.pose.position.y;
    return atan2(vec_y,vec_x);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "goto");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/odometry/filtered", 1000, &odomCB);
    ros::Subscriber laser_sub = nh.subscribe("/scan", 1000, &scanCB);
    ros::Subscriber go_sub = nh.subscribe("vroom", 1000, &goCB);
    ros::Subscriber coordinates = nh.subscribe("/points", 1000, &recievePoints);


    ros::Publisher stopped_pub = nh.advertise<std_msgs::Bool>("/arrived", 1000);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);

    ros::spinOnce();

 bool ROTATING_TOWARD_GOAL = true;
    bool MOVING_TOWARD_GOAL = false;
    bool OBA = false;


    geometry_msgs::Twist motion;
    geometry_msgs::Vector3 random_forward_distance;

    ros::Rate rate(20.0);
    
    
    while(startmoving == false && ros::ok()){
        ROS_INFO("Waiting for coordinates");
        ros::spinOnce();
    }

    ROS_INFO("Starting goal search: Begin Rotating");
    while(startmoving == true && ros::ok()){
        
        ros::spinOnce();
        
        tf2::Quaternion q;
        tf2::convert(curr_robot_pose.pose.pose.orientation, q);
        float current_yaw = tf2::getYaw(q);

        
        

        
            if(ROTATING_TOWARD_GOAL){
                
                ROS_INFO("Rotating toward goal");

                float desired_yaw = calculateDesiredAngle(curr_robot_pose, goal_pose);

                if (std::fabs(desired_yaw - current_yaw) >= THRESHOLD_ANGLE) {
                    ROS_INFO_STREAM("desired_yaw: " << desired_yaw);
                    ROS_INFO_STREAM("current_yaw: " << current_yaw);
                    motion.linear.x = 0.0;
                    motion.angular.z = move_speed;
                    cmd_pub.publish(motion);
                }
                else {

                    motion.linear.x = 0.0;
                    motion.angular.z = 0.0;
                    cmd_pub.publish(motion);

                    if (laser.ranges[390] < distance2D(curr_robot_pose, goal_pose)){
                        ROTATING_TOWARD_GOAL = false;
                        OBA = true;
                    }

                    else{ 
                        ROTATING_TOWARD_GOAL = false;
                        MOVING_TOWARD_GOAL = true;
                    }
                }
                
               
            }  


            if(OBA){

                if (laser.ranges[480] < distance2D(curr_robot_pose, goal_pose)){

                    ROS_INFO("Obstacle Avoidance (Rotating)");

                    motion.linear.x = 0.0;
                    motion.angular.z = move_speed;
                    cmd_pub.publish(motion);
                    
                    random_forward_distance.x = curr_robot_pose.pose.pose.position.x + 5 * std::cos(current_yaw);
                    random_forward_distance.y = curr_robot_pose.pose.pose.position.y + 5 * std::sin(current_yaw);
                }
                else {

                    if (distance2D(curr_robot_pose, random_forward_distance) >= THRESHOLD_GOAL){

                        ROS_INFO("Obstacle Avoidance (Moving forward)");
                        ROS_INFO_STREAM(distance2D(curr_robot_pose, random_forward_distance));

                        motion.linear.x = move_speed;
                        motion.angular.z = 0.0;
                        cmd_pub.publish(motion);
                    }

                    else {
                        OBA = false;
                        ROTATING_TOWARD_GOAL = true;
                    }

                }

                
            }     

            if(MOVING_TOWARD_GOAL){

                

                if (distance2D(curr_robot_pose, goal_pose) >= THRESHOLD_GOAL){
                    ROS_INFO("Moving toward goal");
                    ROS_INFO_STREAM(distance2D(curr_robot_pose, goal_pose));
                    
                    motion.linear.x = move_speed;
                    motion.angular.z = 0.0;
                    cmd_pub.publish(motion);
                }
                
                else {

                    motion.linear.x = 0.0;
                    motion.angular.z = 0.0;
                    ROS_INFO("Goal Reached!");
                    arrivedGoal.data = true;
                    stopped_pub.publish(arrivedGoal);
                    MOVING_TOWARD_GOAL = false;
                    ROTATING_TOWARD_GOAL = true;

                }
                
            }            
           
                
        rate.sleep();
    }
}