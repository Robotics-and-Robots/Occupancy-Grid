#ifndef _HIMM_H
#define _HIMM_H

#include "OccupancyGrid.h"
#include "Vector2D.hpp"
#include "Constants.h"

class Himm{

private:
	OccupancyGrid& _grid;
public:
	Himm(OccupancyGrid& grid);
	~Himm();
}

#endif /* _HIMM_H */
