#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;

geometry_msgs::PoseWithCovarianceStamped amclpose;
geometry_msgs::PoseArray average_pose;

float sumposx = 0, sumposy = 0, sumposz = 0;
float avgposx = 0, avgposy = 0, avgposz = 0;
float sumorix = 0, sumoriy = 0, sumoriz = 0, sumoriw = 0;
float avgorix = 0, avgoriy = 0, avgoriz = 0, avgoriw = 0;


void ParticleCloud(const geometry_msgs::PoseArray::ConstPtr& msg){

    average_pose = *msg;


}

void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

    amclpose = *msg;

}



int main(int argc, char** argv){

    ros::init(argc, argv, "averagepose");
    ros::NodeHandle nh;

    ros::Subscriber ParCloud_sub = nh.subscribe("/particlecloud", 1000, &ParticleCloud);
    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose" , 1000, &amclCB);

    ros::Rate rate(1);

    while(ros::ok()){

        ros::spinOnce();


    //getting all the values for each position and adding them all up
        for(int i = 0; i < average_pose.poses.size(); i++){

            sumposx += average_pose.poses[i].position.x;
            sumposy += average_pose.poses[i].position.y;
            sumposz += average_pose.poses[i].position.z;

            sumorix += average_pose.poses[i].orientation.x;
            sumoriy += average_pose.poses[i].orientation.y;
            sumoriz += average_pose.poses[i].orientation.z;
            sumoriw += average_pose.poses[i].orientation.w;

        }

    //finding the sums for all the positions
    avgposx = sumposx / average_pose.poses.size();
    avgposy = sumposy / average_pose.poses.size();
    avgposz = sumposz / average_pose.poses.size();

    avgorix = sumorix / average_pose.poses.size();
    avgoriy = sumoriy / average_pose.poses.size();
    avgoriz = sumoriz / average_pose.poses.size();
    avgoriw = sumoriw / average_pose.poses.size();


    ROS_INFO("--------------------------------------------------------------------------------------------");

    //print difference between positions
    ROS_INFO_STREAM("Difference of Position X-Coordinate: " << (abs(avgposx - amclpose.pose.pose.position.x)));
    ROS_INFO_STREAM("Difference of Position Y-Coordinate: " << (abs(avgposy - amclpose.pose.pose.position.y)));
    ROS_INFO_STREAM("Difference of Position Z-Coordinate: " << (abs(avgposz - amclpose.pose.pose.position.z)));

    //print difference between orientations
    ROS_INFO_STREAM("Difference of Orientation X-Coordinate: " << (abs(avgorix - amclpose.pose.pose.orientation.x)));
    ROS_INFO_STREAM("Difference of Orientation Y-Coordinate: " << (abs(avgoriy - amclpose.pose.pose.orientation.y)));
    ROS_INFO_STREAM("Difference of Orientation Z-Coordinate: " << (abs(avgoriz - amclpose.pose.pose.orientation.z)));
    ROS_INFO_STREAM("Difference of Orientation W-Coordinate: " << (abs(avgoriw - amclpose.pose.pose.orientation.w)));



    sumposx = 0;
    sumposy = 0;
    sumposz = 0;
    sumoriw = 0;
    sumorix = 0;
    sumoriy = 0;
    sumoriz = 0;

    rate.sleep();

    }


}