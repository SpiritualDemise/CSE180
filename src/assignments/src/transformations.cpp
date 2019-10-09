#include<ros/ros.h>
#include <nav_msgs/Odometry.h>
#include<math.h>
#include<geometry_msgs/TransformStamped.h>

geometry_msgs::TransformStamped bl_bs;
geometry_msgs::TransformStamped rota;

void tfStatCB(const tf2_msgs::TFMessage& msg){

	bl_bs` = msg.transforms[i];

}

//void rotationMSG(const tf2_msgs::TFMessage& msg){

//	rota = msg.transform.rotation;

//}


int main(int argc, char** argv){

ros::init(argc,argv,"transformations");

ros::NodeHandle nh;

ros::Subsriber subtrans = nh.subscribe("/tf_static",1000,&transformationMSG);
//ros::Subsriber subrota = nh.subscribe("/tf", 1000,&rotationMSG);

ros::Rate rate(1);

//tf2_msgs::TFMessage fs_bl;
//tf2_msgs:: TFMessage bs_bl;

fs_bl.transform[i].transform.translation.x = ;

while(ros::ok()){





}


}


