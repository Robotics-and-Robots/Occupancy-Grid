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
OGCellType OccupancyGrid::Set(int x, int y, OGCellType cvalue){
	
	int a = round(std::abs(x));
	int b = round(std::abs(y));

	if(a >= OG_SEC_W) return 0;
	if(b >= OG_SEC_H) return 0;

	//return values for x-positive slice of grid
	if( x >= 0){
		if( y >= 0){
			if(cvalue > 0 ){
				if(_m_pospos[a][b] < HIMM_THRESHOLD_MAX){	// increment
					return _m_pospos[a][b] += cvalue;
				}	
			}else {
				if(_m_pospos[a][b] > HIMM_THRESHOLD_MIN){	// decrement
					return _m_pospos[a][b] += cvalue;
				}	
			}
				
			
		}else{ // y < 0
			if(cvalue > 0 ){
				if(_m_posneg[a][b] < HIMM_THRESHOLD_MAX){	// increment	
					return _m_posneg[a][b] += cvalue;
				}	
			}else {
				if(_m_posneg[a][b] > HIMM_THRESHOLD_MIN){	// decrement
					return _m_posneg[a][b] += cvalue;
				}	
			}
		}
	}else{	// x < 0
		if( y >= 0){
			if(cvalue > 0 ){
				if(_m_negpos[a][b] < HIMM_THRESHOLD_MAX){	// increment
					return _m_negpos[a][b] += cvalue;
				}	
			}else {
				if(_m_negpos[a][b] > HIMM_THRESHOLD_MIN){	// decrement
					return _m_negpos[a][b] += cvalue;
				}	
			}
				
			
		}else{ // y < 0
			if(cvalue > 0 ){
				if(_m_negneg[a][b] < HIMM_THRESHOLD_MAX){	// increment
					return _m_negneg[a][b] += cvalue;
				}	
			}else {
				if(_m_negneg[a][b] > HIMM_THRESHOLD_MIN){	// decrement
					return _m_negneg[a][b] += cvalue;
				}	
			}
		}
	}
}

/**
 * Set all cell values to zeroes
 */
void OccupancyGrid::Reset(){
	for(int y = OG_SEC_H; y > -OG_SEC_H; y--){
		for(int x = -OG_SEC_W; x < OG_SEC_W; x++){

			double curr_val;
			curr_val = this->Get(x, y);
			curr_val = (curr_val > 0) ? (curr_val - 1) : 0;
			this->Set(x, y, curr_val);
		}
	}
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
                   << "stroke='#" << std::hex << (this->Get(x, y) * (255 / (HIMM_THRESHOLD_MAX - HIMM_THRESHOLD_MIN))) << "FFFF' />";
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
