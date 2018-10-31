#include <iostream>										// cout

#include "ros/ros.h"									// ROS
#include "sensor_msgs/LaserScan.h"						// Hokuyo laser msgs
#include "geometry_msgs/Twist.h"						// Twist - message for motion
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"

#include "../include/OccupancyGrid.h"					// Grid class

#define HOKUYO_NUM_RANGES 683

// Functions
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
void move(double, double , bool);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);				// callback that updates current location (from odometry)
void toEulerAngle(double x, double y, double z, double w, double* yaw);	// Function to convert from quarternion to euler angle 


//Global Variables
ros::Publisher velocity_publisher;
OccupancyGrid* g;														//global occupancy grid
geometry_msgs::Pose2D _pos;												//global position vector

/*
*
*	@Author: 		Anderson Domingues e Darlan Alves Jurak
*	@Brief: 		Main function.
* 	@Description:	Responsable for robot motion control and mapping filling.
*
*/
int main(int argc,char **argv)
{
	ros::init(argc,argv,"OccupancyGridNode");
	ros::NodeHandle n;

	// g->Set(7, 7, 54.5f);
	// std::cout << g->Get(7, 7) << std::endl;

	ros::Subscriber subScan	= n.subscribe("/scan", 1000, processLaserScan);						// Listening Hokuyo laser ranges info
	ros::Subscriber subOdom = n.subscribe("odom", 1000,  odomCallback);							//data from other nodes 
	velocity_publisher 		= n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);	// Defines robot motion publisher

 	g = new OccupancyGrid();

	move(1, 10, 1);

	//Let ROS take over
	ros::spin();

	return 0;
}

/*
*
*	@Author: 		Anderson Domingues e Darlan Alves Jurak
*	@Brief: 		Hokuyo range info "print"
*
*/
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	ROS_INFO("position=: [%f]", scan->ranges[270]);

	//update uccupancy grid for all ranges
	for(int i = 0; i < HOKUYO_NUM_RANGES; i++)
		g->SetLoc(
			_pos.x, 
			_pos.x, 
			scan->ranges[i], 
			(i * 0.36)
		);

	//print occupancy grid
	// ROS_INFO(g->ToString().c_str());
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

/*
*
*	@Author: 		Anderson Domingues
*	@Brief: 		Main function.
* 	@Description:	Responsable for update current location (from odometry)
*
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

	//update location
	_pos.x 		= msg->pose.pose.position.x;
	_pos.y 		= msg->pose.pose.position.y;

	//update angle
	toEulerAngle(
		msg->pose.pose.orientation.x, 
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w,
		&_pos.theta
	);

}

// Function to convert from quarternion to euler angle 
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles 
void toEulerAngle(double x, double y, double z, double w, double* yaw){
	double siny = +2.0 * (w * z + x * y);
	double cosy = +1.0 - 2.0 * (y * y + z * z);  
	*yaw = atan2(siny, cosy);
}