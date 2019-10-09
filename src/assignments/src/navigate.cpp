#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Bool.h"
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>


geometry_msgs::PoseWithCovarianceStamped currpose;

void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){

    currpose = *msg;

}

float distance2D(geometry_msgs::PoseWithCovarianceStamped pose1, geometry_msgs::PoseStamped pose2) {
    float x = pose2.pose.position.x - pose1.pose.pose.position.x;
    float y = pose2.pose.position.y - pose1.pose.pose.position.y;
    return hypotf(x, y);
}



int main(int argc, char** argv){

    ros::init(argc, argv, "navigate");
    ros::NodeHandle nh;


    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1000, &amclCB);
    
    ros::Publisher given_point = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    
    int numpositions = 4;
    float values[] = {-5,-5, -5,5, 5,5, 5,-5};

    geometry_msgs::PoseStamped destination;
    geometry_msgs::PoseStamped oldpose;
    
    int i = 0;
    int j = 1;
    int oof = 0;

    destination.pose.position.x = values[i];
    destination.pose.position.y = values[j];
    destination.pose.orientation.w = 1;         
    destination.header.frame_id = "map";
    ros::Duration wait(4.0);
    wait.sleep();

    ros::Rate rate(1);
    given_point.publish(destination);

    oldpose.pose.position.x = destination.pose.position.x;
    oldpose.pose.position.y = destination.pose.position.y;




    while(ros::ok()){


        ros::spinOnce();

        if(distance2D(currpose, oldpose) == 0){

            
            if(distance2D(currpose, destination) <= 0.3){
                oof = 0;
            i = (i + 2) % (numpositions * 2);
            j = (j + 2) % (numpositions * 2);

            destination.pose.position.x = values[i];
            destination.pose.position.y = values[j];

            given_point.publish(destination);
            wait.sleep();

            }
            else{
                oof++;
            destination.pose.position.x = values[i];
            destination.pose.position.y = values[j];

            given_point.publish(destination);
            wait.sleep();

            

            if(oof >= 3){

                break;
            }

            }

        }

        oldpose.pose.position.x = currpose.pose.pose.position.x;
        oldpose.pose.position.y = currpose.pose.pose.position.y;
        

        rate.sleep();

    }


}