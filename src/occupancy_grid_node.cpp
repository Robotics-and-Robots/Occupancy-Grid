#include <iostream>										//cout
#include "ros/ros.h"									//ros
#include "sensor_msgs/LaserScan.h"						// hokuyo msgs
#include "../include/occupancy_grid/OccupationGrid.hpp"	//grid



void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);


int main(int argc, char** argv){

	ros::NodeHandle nh;
	ros::Subscriber scanSub;

	scanSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &processLaserScan);

	OccupationGrid* g = new OccupationGrid();

	g->Set(7, 7, 54.5f);

	std::cout << g->Get(7, 7) << std::endl;

	return 0;
}

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
    //scan->ranges[] are laser readings
}