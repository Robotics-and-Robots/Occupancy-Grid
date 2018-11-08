#ifndef _POTENTIAL_FIELDS_H
#define _POTENTILA_FIELDS_H

#include "OccupancyGrid.h"

class PotentialFields{

private:
	OccupancyGrid* _grid;
	OccupancyGrid* _temp_grid;
public:
	PotentialFields(OccupancyGrid* grid);
	~PotentialFields();

	/** Get the next coordinate to walk given current
      * location and a target location */
	Pose2D GetNextPosition(Pose2D pose, Pose2D);
	
	void ToFile(std::string filename);

	void UpdateRoutes();
};

#endif
