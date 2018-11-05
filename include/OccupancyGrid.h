#ifndef OCCUPATION_GRID_H
#define OCCUPATION_GRID_H

#include <cmath>
#include "geometry_msgs/Pose2D.h"

using namespace geometry_msgs;

#define UNIT_FIX 150 /* means 1 unit from hokuyo scales up to X while in the grid */

#define OG_WIDTH (2 * UNIT_FIX)
#define OG_HEIGHT (2 * UNIT_FIX)

#define OG_SEC_W (OG_WIDTH  / 2)
#define OG_SEC_H (OG_HEIGHT / 2)

//Cell elements are float-typed values
typedef double OGCellType;

//Defines a class to hold values for an
//occupation grid. 
class OccupancyGrid{

private:
	OGCellType _m_pospos[OG_SEC_W][OG_SEC_H]; //X is positive, Y is positive.
	OGCellType _m_posneg[OG_SEC_W][OG_SEC_H]; //X is positive, Y is negative.
	OGCellType _m_negpos[OG_SEC_W][OG_SEC_H]; //X is negative, Y is positive.
	OGCellType _m_negneg[OG_SEC_W][OG_SEC_H]; //X is negative, Y is negetive.

public:

	//ctor. and dtor.
	OccupancyGrid();
	~OccupancyGrid();
	
	//overload [] to access as an array
	OGCellType Get(int x, int y);
	
	//set some value to a cell
	OGCellType Set(int x, int y, OGCellType value);
	OGCellType SetLoc(Pose2D pose, OGCellType dist, OGCellType theta);

	void Reset();

	std::string ToString();
	void ToFile(std::string filename);
};

#endif /* OCCUPATION_GRID_H */
