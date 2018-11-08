#include "../Himm.h"

/**
 * Ctor.
 * @grid Reference to the grid in which the hmmi
 * will read and write values.
 */
Himm::Himm(OccupancyGrid& grid){
	this->_grid = grid;
}

Himm::ToFile(std::string filename){
	this->_grid->ToFile(filename);
}

Himm::~Himm(){}

/**
 * Set a value to the grid according to the position of the robot
 * @param pose A Pose2D struct representing the position and rotation of the robot
 * @param dist The distance captured by the sensor
 * @param theta The inclination of the laser ray during the reading in degrees
 * @returns The value written to the cell
 */
OGCellType Himm::UpdateLocation(Pose2D pose, OGCellType dist, OGCellType theta){

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

	OGCellType hDist = cos(odom_theta) * dist; //horizontal distance
	OGCellType wDist = sin(odom_theta) * dist; //vertical distance

	int x_coord = (pose.x + hDist) * UNIT_FIX;
	int y_coord = (pose.y + wDist) * UNIT_FIX;


	//decrement cells in the path
	/*Vector2D vec;
	vec.x = ((pose.x + hDist) * UNIT_FIX) - pose.x;
	vec.y = ((pose.y + wDist) * UNIT_FIX) - pose.y;

	Vector2D vecUnitary = ~vec;

	Vector2D posev;
	posev.x = pose.x;
	posev.y = pose.y;
	
	int i = 1;
	Vector2D curr;
	do{

		curr = (vecUnitary * i) + posev;
		this->Set(curr.x, curr.y, -1);
		i++;

	}while(!curr < !vec);
*/

	//set location with target increment
	_grid->Set(x_coord, y_coord, 1);
}

