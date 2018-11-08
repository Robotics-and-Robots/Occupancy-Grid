#include "../include/PotentialFields.h"

void PotentialFields::UpdateRoutes(){

}

PotentialFields::PotentialFields(OccupancyGrid* grid){
	this->_grid = grid;
}

PotentialFields::~PotentialFields(){}

/** Get the next coordinate to walk given current
  * location and a target location */
Pose2D PotentialFields::GetNextPosition(Pose2D pose, Pose2D){
	//algorithm here
}

void PotentialFields::ToFile(std::string filename){

}
