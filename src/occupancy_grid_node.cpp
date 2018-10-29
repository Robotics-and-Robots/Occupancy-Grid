#include <iostream>										// cout
#include "ros/ros.h"									// ROS
#include "sensor_msgs/LaserScan.h"						// Hokuyo laser msgs
#include "geometry_msgs/Twist.h"						// Twist - message for motion
#include "../include/occupancy_grid/OccupationGrid.hpp"	// Grid class

// Functions
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
void move(double, double , bool);

//Global Variables
ros::Publisher velocity_publisher;

/*
*
*	@Author: 		Anderson Domingues e Darlan Alves Jurak
*	@Brief: 		Main function.
* 	@Description:	Responsable for robot motion control and mapping filling.
*
*/
int main(int argc,char **argv)
{
	ros::init(argc,argv,"occupancy_grid_node");
	ros::NodeHandle n;

	OccupationGrid* g = new OccupationGrid();

	// g->Set(7, 7, 54.5f);
	// std::cout << g->Get(7, 7) << std::endl;

	// Listening Hokuyo laser ranges info
	ros::Subscriber sub = n.subscribe("/scan", 1000, processLaserScan);

	// Defines robot motion publisher
	velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);

	move(1, 10, 1);

	//Let ROS take over
	ros::spin();

	return 0;
}

/*
*
*	@Author: 		Darlan Alves Jurak
*	@Brief: 		Hokuyo range info "print"
*
*/
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	ROS_INFO("position=: [%f]", scan->ranges[270]);
}

/*
*
*	@Author: 		Darlan Alves Jurak
*	@Brief: 		Basic motion function.
* 	@Description:	Forward and backward motion based on desired velocity and distance.
*
*/
void move(double speed, double distance, bool isForward){

	// Motion message (angular and linear x, y and z )
	geometry_msgs::Twist vel_msg;

	// Set linear velocity in the x-axis
	if(isForward){

		vel_msg.linear.x = abs(speed);

	}else{

		vel_msg.linear.x = -abs(speed);

	}

	// Anulates velocity in "y" and "z" axis 
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	double t0 = ros::Time::now().toSec();		// Base time for comparison
	double current_distance = 0;				
	ros::Rate loop_rate(10);
	do{

		velocity_publisher.publish(vel_msg);	// Moves robot
		double t1 = ros::Time::now().toSec();	// Get current time
		current_distance = speed * (t1 - t0);	// Updates travelled distance estimation

		loop_rate.sleep();

	}while( current_distance < distance);

	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);

}