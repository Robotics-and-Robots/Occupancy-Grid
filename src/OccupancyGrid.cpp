#include <sstream>
#include "../include/OccupancyGrid.h"


using namespace geometry_msgs;

OccupancyGrid::OccupancyGrid(){}
OccupancyGrid::~OccupancyGrid(){}

OGCellType OccupancyGrid::Get(int x, int y){

	int a = std::abs(x);
	int b = std::abs(y);

	if(x >=0 && y >=0) return _m_pospos[a][b];
	if(x >=0 && y < 0) return _m_posneg[a][b];
	if(x < 0 && y >=0) return _m_negpos[a][b];
	else return _m_negneg[a][b];
}

OGCellType OccupancyGrid::Set(int x, int y, OGCellType value){
	
	int a = std::abs(x);
	int b = std::abs(y);

	if(x >=0 && y >=0) return _m_pospos[a][b] = value;
	if(x >=0 && y < 0) return _m_posneg[a][b] = value;
	if(x < 0 && y >=0) return _m_negpos[a][b] = value;
	else return _m_negneg[a][b] = value;
}

OGCellType OccupancyGrid::SetLoc(Pose2D pose, OGCellType dist, OGCellType theta){

	OGCellType hDist = cos(theta + pose.theta) * dist; //horizontal distance
	OGCellType vDist = sin(theta + pose.theta) * dist; //vertical distance

	this->Set(
		pose.x + hDist,
		pose.y + vDist,
		1
	);
}

std::string OccupancyGrid::ToString(){

	std::stringstream ss;

	//( -x, +y)
	for(int y = OG_SEC_H; y > -OG_SEC_H; y--){

		for(int x = -OG_SEC_H; x < OG_SEC_H; x++)
			ss << this->Get(x, y) << " ";

		ss << std::endl;
	}

	return ss.str();

}
