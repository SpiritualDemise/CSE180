#include <ros/ros.h>
#include <move_base/move_base.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

nav_msgs::OccupancyGrid localmapvalue;
nav_msgs::OccupancyGrid globalmapvalue;



void localcostmap(const nav_msgs::OccupancyGrid& msg){

    localmapvalue = msg;

    ROS_INFO_STREAM("Received Local Map");
    ROS_INFO_STREAM("Dimensions: " <<msg.info.width << " x " << msg.info.height);
    ROS_INFO_STREAM("Resolution " << msg.info.resolution);
    std::cout << "--------------------------------------------------------------------------------------------------------------" << std::endl;

}

void globalcostmap(const nav_msgs::OccupancyGrid&msg) {

    globalmapvalue = msg;

    ROS_INFO_STREAM("Received Global Map");
    ROS_INFO_STREAM("Dimensions: " <<msg.info.width << " x " << msg.info.height);
    ROS_INFO_STREAM("Resolution " << msg.info.resolution);
    ROS_INFO_STREAM("Saving costmap to a file");

    std::cout << "--------------------------------------------------------------------------------------------------------------" << std::endl;

    std::ofstream globalmap("map.txt");
    int z= 0;
    for (int i = 0 ; i < msg.info.height ; i++) {
      for ( int j = 0 ; j < msg.info.width ; j++ )
	  globalmap << (int)msg.data[z++] << " " ;
      globalmap << std::endl;
  }
  globalmap.close();

}

int main(int argc,char ** argv) {

  ros::init(argc,argv,"projectscan");
  ros::NodeHandle nh;

  ros::Subscriber subglobal = nh.subscribe("/move_base/global_costmap/costmap", 100,&globalcostmap);
  ros::Subscriber sublocal = nh.subscribe("/move_base/local_costmap/costmap", 100,&localcostmap); 

  int differentofmap[globalmapvalue.data.size()];

  for(int i = 0; i < globalmapvalue.data.size(); i++){

    differentofmap[i] = globalmapvalue.data[i] - localmapvalue.data[i];

    std::cout << "--------------------------------------------------------------------------------------------------------------" << std::endl;

    ROS_INFO_STREAM("difference in cost map :" << differentofmap[i]);

    std::cout << "--------------------------------------------------------------------------------------------------------------" << std::endl;

  }



  ros::spin();

}