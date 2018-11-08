/*
*
*	@Author: 		Anderson Domingues e Darlan Alves Jurak
*	@Brief: 		Main ROS node for Occupancy Grid generation
*
*/

#include <iostream>										// cout

#include "ros/ros.h"									// ROS
#include "sensor_msgs/LaserScan.h"						// Hokuyo laser msgs
#include "geometry_msgs/Twist.h"						// Twist - message for motion
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"

//configurations
#include "../include/Constants.h"

//algorithms and data structures
#include "../include/OccupancyGrid.h"
#include "../include/PotentialFields.h"
#include "../include/Hmmi.h"

using namespace geometry_msgs;

//Prototypes
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
void move(double, double , bool);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); 
double toEulerAngle(double x, double y, double z, double w);

//Global Variables
ros::Publisher pub_velocity;

//global occupancy grid, hmmi and potential fields
OccupancyGrid& _occupancy_grid;
PotentialFields& _potential_fields;
Hmmi& _hmmi;
														
//global position vector (updated in odom callback)
geometry_msgs::Pose2D _pos;

/**
 * Responsable for robot motion control and mapping filling.
 * @author Anderson Domingues and Darlan Alves Jurak
 */
int main(int argc,char **argv)
{
	ros::init(argc,argv,"OccupancyGridNode");
	ros::NodeHandle n;

	//subscribe to Hokuyo node (laser)
	ros::Subscriber sub_scan = n.subscribe("/scan", 100, processLaserScan);

	//subscribe to Odom node (odometry and movement)
	ros::Subscriber sub_odom = n.subscribe("/odom", 1000, odomCallback);

	// Defines robot motion publisher
	pub_velocity = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);	

	//instantiates a new occupancy grid, hmmi and potential fields algorithms
 	_occupancy_grid = OccupancyGrid();
    _hmmi = Hmmi(_occupancy_grid);
	_potential_fields = PotentialFields(_occupancy_grid);

	// Let ROS take over
	ros::spin();

	return 0;
}

/*using namespace geometry_msgs;
*
*	@Author: 		Anderson Domingues e Darlan Alves Jurak
*	@Brief: 		Hokuyo range info "print"
*
*/
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	double reading; 

	//set all values from grid to zeroes
	//g->Reset();

	//ROS_INFO("min_angle [%f] max_angle [%f]", scan->angle_min, scan->angle_max);
	//ROS_INFO("angle_increment [%f]", scan->angle_increment);
	//ROS_INFO("range_min [%f] range_max [%f]", scan->range_min, scan->range_max);
	
	//update uccupancy grid for all ranges
	double i = HOKUYO_ANGLE_MAX;
	int j = 0;
	while(i >= HOKUYO_ANGLE_MIN){
	 	
		reading = scan->ranges[j];

		if(reading <= HOKUYO_RANGE_MAX && reading >= HOKUYO_RANGE_MIN && !isnan(reading)){
		
			g->SetLoc(
				_pos,
				reading, //value read
				i //angle
			);
		}

		i -= HOKUYO_ANGLE_INC;
		j++;
	}

	//save occupancy grid to file
	g->ToFile("/home/lsa/Desktop/filename.html");
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

		pub_velocity.publish(vel_msg);	// Moves robot
		double t1 = ros::Time::now().toSec();	// Get current time
		current_distance = speed * (t1 - t0);	// Updates travelled distance estimation

		loop_rate.sleep();

	}while( current_distance < distance);

	vel_msg.linear.x = 0;
	pub_velocity.publish(vel_msg);

}

/*
*	@Author: 		Anderson Domingues
*	@Brief: 		Main function.
* 	@Description:	Responsable for update current location (from odometry)
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

	//update location
	_pos.x 		= msg->pose.pose.position.x;
	_pos.y 		= msg->pose.pose.position.y;

	//update angle
	_pos.theta = toEulerAngle(
		msg->pose.pose.orientation.x, 
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w //already in radians
	);

	//ROS_INFO("position=: [%f] [%f] ([%f])", _pos.x, _pos.y, _pos.theta);
}

// 
/**
 * Function to convert from quarternion to euler angle 
 * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles 
 * @param x Quarternion.x
 * @param y Quarternion.y
 * @param z Quarternion.z
 * @param w Quarternion.w
 * @param yaw Pointer to the variable to be written with the eulerian 
 * value associated with the given quarternion */
double toEulerAngle(double x, double y, double z, double w){
	double siny = +2.0 * (w * z + x * y);
	double cosy = +1.0 - 2.0 * (y * y + z * z);  
	return atan2(siny, cosy); 
}
