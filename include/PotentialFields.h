#ifndef _POTENTIAL_FIELDS_H
#define _POTENTILA_FIELDS_H

#include "OccupancyGrid.h"

class PotentialFields{

private:
	OccupancyGrid& _grid;
	OccupancyGrid& _temp_grid;
	updateTempGrid();
public:
	PotentialFields(OccupancyGrid& grid);
	~PotentialFields();

	/** Get the next coordinate to walk given current
      * location and a target location */
	GetNextPosition(Pose2D pose, Pose2D);


}

#endif
