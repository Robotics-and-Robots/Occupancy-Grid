#include <sstream>
#include <fstream>
#include <cmath>
#include "../include/OccupancyGrid.h"
#include "ros/ros.h"

using namespace geometry_msgs;

OccupancyGrid::OccupancyGrid(){}
OccupancyGrid::~OccupancyGrid(){}


/**
 * Retuns the value of some cell into the grid
 * @param x x-coordinate of the cell to be read
 * @param y y-coordinate of the cell to be read
 * @returns Value of the read cell
 */
OGCellType OccupancyGrid::Get(int x, int y){

	int a = std::abs(x);
	int b = std::abs(y);

	if(a > OG_SEC_W) return 0;
	if(b > OG_SEC_H) return 0;

	//return values for x-positive slice of grid
	if(x >= 0) return (y >= 0) ? (_m_pospos[a][b]) : (_m_posneg[a][b]);

	//return values for x-negative slice of grid
	return (y >= 0) ? (_m_negpos[a][b]) : (_m_negneg[a][b]);
}

/**
 * Set a value to some cell in the occupancy grid
 * @param x x-coordinate of cell
 * @param y y-coordinate of cell
 * @param value value to be written into the cell
 * @returns the value written to the cell (must be the same as value param)
 */
OGCellType OccupancyGrid::Set(int x, int y, OGCellType value){
	
	int a = std::abs(x);
	int b = std::abs(y);

	if(a >= OG_SEC_W) return 0;
	if(b >= OG_SEC_H) return 0;

	//return values for x-positive slice of grid
	if(x >= 0) return (y >= 0) ? (_m_pospos[a][b] = value) : (_m_posneg[a][b] = value);

	//return values for x-negative slice of grid
	return (y >= 0) ? (_m_negpos[a][b] = value) : (_m_negneg[a][b] = value);
}

/**
 * Set a value to the grid according to the position of the robot
 * @param pose A Pose2D struct representing the position and rotation of the robot
 * @param dist The distance captured by the sensor
 * @param theta The inclination of the laser ray during the reading in degrees
 * @returns The value written to the cell
 */
OGCellType OccupancyGrid::SetLoc(Pose2D pose, OGCellType dist, OGCellType theta){

	double odom_theta;
	if(pose.theta < 0)
		odom_theta = (2 * M_PI) - std::abs(pose.theta);
	else 
		odom_theta = pose.theta;

	double calc_theta;
	if(theta < 0)
		calc_theta = (2 * M_PI) - std::abs(theta);
	else
		calc_theta = theta;

	//angle correction (neg to pos rad values)
	odom_theta -= calc_theta;

	if(odom_theta < 0)
		odom_theta  = (2 * M_PI) - std::abs(odom_theta);

	//scale correction
	//pose.x = pose.x * UNIT_FIX;
	//pose.y = pose.y * UNIT_FIX;

	OGCellType hDist = cos(odom_theta) * dist; //horizontal distance
	OGCellType vDist = sin(odom_theta) * dist; //vertical distance

	//ROS_INFO("position x=[%f] y=[%f] angle=[%f] dist=[%f], theta=[%f]",
		//pose.x + hDist, pose.x + hDist, pose.theta, dist, theta);

	return this->Set(
		(pose.x + hDist) * UNIT_FIX,
		(pose.y + vDist) * UNIT_FIX,
		1
	);
}



/**
 * Set all cell values to zeroes
 */
void OccupancyGrid::Reset(){
	for(int y = OG_SEC_H; y > -OG_SEC_H; y--)
		for(int x = -OG_SEC_W; x < OG_SEC_W; x++)
			this->Set(x, y, 0);
}

/**
 * Export current grid values to a HTML file containing a SGV drawing.
 * @param filaname Path to the file to be written.
 */
void OccupancyGrid::ToFile(std::string filename){

	std::ofstream of;
	std::stringstream ss;

	ss << "<html>\n"
       << "<head>\n"
       << "<meta http-equiv=\"refresh\" content=\"1\" />\n"
       << "</head>\n"
	   << "<body style='background:black;'>\n"
	   << "<div style='text-align:center;margin:auto;'>\n"
       << "<h3 style='color:white;font-family:sans'>Occupancy Grid</h3><br />\n"
	   << "<svg style='width:" << (OG_WIDTH*4) << "px; height:" << (OG_HEIGHT*4) << "px; diaply:block; margin:auto'>\n";
	int w = 0; 
	int h = 0;
	for(int y = OG_SEC_H; y > -OG_SEC_H; y--){
		for(int x = -OG_SEC_W; x < OG_SEC_W; x++){

			if (this->Get(x, y) > 0)
			    ss << "<circle cx='" << (w*4) << "' cy='" << (h*4) << "' r='1' stroke-width='3' " 
                   << "stroke='green' />";
			else if(x == 0 || y == 0)
				ss << "<circle cx='" << (w*4) << "' cy='" << (h*4) << "' r='1' stroke-width='1' " 
                   << "stroke='gray' />";
			w++;
		}
		h++;
		w = 0;
		ss << std::endl;
	}

	ss << "</div></svg>"
       << "</body>"
       << "</html>";

	of.open(filename.c_str(), std::ofstream::trunc);
	of << ss.str();
	of.close();
}
