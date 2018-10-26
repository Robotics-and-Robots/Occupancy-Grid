#include <iostream>										// cout
#include "ros/ros.h"									// ros
#include "sensor_msgs/LaserScan.h"						// hokuyo laser msgs
#include "../include/occupancy_grid/OccupationGrid.hpp"	// grid

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	ROS_INFO("position=: [%f]", scan->ranges[270]);
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"occupancy_grid_node");
	ros::NodeHandle n;

	OccupationGrid* g = new OccupationGrid();

	// g->Set(7, 7, 54.5f);
	// std::cout << g->Get(7, 7) << std::endl;

	//Create a subscriber object
	ros::Subscriber sub = n.subscribe("/scan",1000, processLaserScan);

	//Let ROS take over
	ros::spin();

	return 0;
}