#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>

bool destinationcomplete;
std_msgs::Bool comense;

void destinationcompleteCB(const std_msgs::Bool &msg){

    destinationcomplete = msg.data;

}

int main(int argc, char** argv){

    ros::init(argc, argv, "tour");
    ros::NodeHandle nh;

    ros::Subscriber Location = nh.subscribe("/arrived", 1000, &destinationcompleteCB);
    ros::Publisher coordinates = nh.advertise<geometry_msgs::Vector3>("/points", 1000);
    ros::Publisher vroom = nh.advertise<std_msgs::Bool>("vroom", 1000);
    
    int numpositions = 3;
    float values[] = {1,2,5,6,0,7};

    geometry_msgs::Vector3 positions;

    int i = 0;
    int j = 1;

    positions.x = values[i];
    positions.y = values[j];

    comense.data = true;
    
    ros::spinOnce();
    ROS_INFO_STREAM("X-Coordinate: " << positions.x);
    ROS_INFO_STREAM("Y-Coordinate: " << positions.y);

    ros::Rate rate(20);

    while(ros::ok()){

        vroom.publish(comense);
        coordinates.publish(positions);

        ros::spinOnce();

        if(destinationcomplete){

            ROS_INFO_STREAM("X-Coordinate: " << positions.x);
            ROS_INFO_STREAM("Y-Coordinate: " << positions.y);

            i = i + 2;
            j = j + 2;

            positions.x = values[i];
            positions.y = values[j];

            coordinates.publish(positions);
            destinationcomplete = false;



        }

    if( i % 6 == 0){

        i = 0;
        j = 1;

    }

    rate.sleep();

    }
    
}