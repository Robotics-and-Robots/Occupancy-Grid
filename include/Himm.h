#ifndef _HIMM_H
#define _HIMM_H

#include "std_msgs::Pose2D"

#include "OccupancyGrid.h"
#include "Vector2D.hpp"
#include "Constants.h"

class Himm{

private:
	OccupancyGrid& _grid;
public:
	Himm(OccupancyGrid& grid);
	OGCellType UpdateLocation(Pose2D pose, OGCellType dist, OGCellType theta);
	~Himm();
}

#endif /* _HIMM_H */
