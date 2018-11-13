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
	
	int a = std::abs(x);
	int b = std::abs(y);

	if(a >= OG_SEC_W) return 0;
	if(b >= OG_SEC_H) return 0;

	double newValue = this->Get(x, y) + cvalue;

	if(newValue < HIMM_THRESHOLD_MIN) return 0;
	if(newValue > HIMM_THRESHOLD_MAX) return 0;
	
	if(x >= 0) 
		return (y >= 0) ? (_m_pospos[a][b] = newValue) : (_m_posneg[a][b] = newValue);
	else
		return (y >= 0) ? (_m_negpos[a][b] = newValue) : (_m_negneg[a][b] = newValue);
}

OGCellType OccupancyGrid::Set_PF(int x, int y, OGCellType cvalue){
	
	int a = std::abs(x);
	int b = std::abs(y);

	if(a >= OG_SEC_W) return 0;
	if(b >= OG_SEC_H) return 0;
	
	if(x >= 0) 
		return (y >= 0) ? (_m_pospos[a][b] = cvalue) : (_m_posneg[a][b] = cvalue);
	else
		return (y >= 0) ? (_m_negpos[a][b] = cvalue) : (_m_negneg[a][b] = cvalue);
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
		<< "<meta http-equiv=\"refresh\" content=\"2\" />\n"
		<< "</head>\n"
	   	<< "<body style='background:black;'>\n"
	   	<< "<div style='text-align:center;margin:auto;'>\n"
		<< "<h3 style='color:white;font-family:sans'>Occupancy Grid</h3><br />\n"
		<< "<svg style='width:" << OG_WIDTH << "px; height:" << OG_HEIGHT << "px; display:block; margin:auto'>\n";

	int w = 0; 
	int h = 0;

	//from the maximum Y to the minimum Y, do
	for(int y = OG_SEC_H; y > -OG_SEC_H; y--){

		//from the minimum X to the maximum X, do
		for(int x = -OG_SEC_W; x < OG_SEC_W; x++){

			//convert cell values to a grayscale color
			uint32_t gscolor = this->Get(x, -y);
			
			//ignore cell without any value
			if (gscolor > 1){
				
				//convert cell value to grayscale
				gscolor = gscolor * (255 / (HIMM_THRESHOLD_MAX - HIMM_THRESHOLD_MIN));

				//add a pixel with the generated color to the bytestream
				ss << std::dec << "<rect x='" << (x + OG_SEC_W) << "' y='" << (y + OG_SEC_H) << "' width=1 height=1 ";
				ss << " fill='#" << std::hex << gscolor << gscolor << gscolor << "' /> ";
			}

			w++;
		}

		h++;
		w = 0;
	}


	ss << "<line x1='" << std:: dec << 0 << "' y1='" << OG_HEIGHT/2 << "' x2='" << OG_WIDTH << "' y2='" << OG_HEIGHT/2 << "' stroke=red />";
	ss << "<line x1='" << OG_WIDTH/2  << "' y1='0' x2='" << OG_WIDTH/2 << "' y2='" << OG_HEIGHT << "' stroke=red />";


	ss << "</svg></div>"
		<< "</body>"
		<< "</html>";

	of.open(filename.c_str(), std::ofstream::trunc);
	of << ss.str();
	of.close();
}

std::string OccupancyGrid::ToMap(){

	std::stringstream ss; 
	double temp_val;

	for(int i = -OG_SEC_W; i < OG_SEC_W; i++){
		for(int j = -OG_SEC_H; j < OG_SEC_H; j++){
			temp_val = this->Get(i,j);

			if(temp_val >= PF_THRESHOLD)
				ss << "1 ";
			else
				ss << "0 ";

		}
		ss << std::endl;
	}

	return ss.str();

}

void OccupancyGrid::LoadMap(std::string filename){

	std::ifstream in(filename.c_str());
	double temp_val;

	if (!in) {
		ROS_INFO("Cannot open file.");
    	return;
  	}

	for(int i = -OG_SEC_W; i < OG_SEC_W; i++){
		for(int j = -OG_SEC_H; j < OG_SEC_H; j++){
			in >> temp_val;

			if(temp_val != 0)
				this->Set(i,j, HIMM_THRESHOLD_MAX - 1);	
		}
	}

	in.close();

}

void OccupancyGrid::UpdatePotentialFields(){

	Vector2D goal;
	goal.x = 1;
	goal.y = 1;

	OccupancyGrid* tempGrid;
	tempGrid = new OccupancyGrid();

	//repete algorithm k vezes
	for(int k = 0; k < PF_ITERATIONS; k++){

		//popula segunda grid
		for(int i = -OG_SEC_W; i < OG_SEC_W; i++){
			for(int j = -OG_SEC_H; j < OG_SEC_H; j++){
				double currentValue;
				currentValue = this->Get(i, j);

				//eh parede
				if(currentValue == 1){
					tempGrid->Set_PF(i, j, 1);

				//eh objetivo
				}else if(i == goal.x && j == goal.y){
					tempGrid->Set_PF(i, j, 0);

				//eh qualquer outra coisa
				}else{
					double meanNeighs = (
						this->Get(i + 1, j) +
						this->Get(i - 1, j) +
						// this->Get(i + 1, j + 1) +
						// this->Get(i - 1, j + 1) +
						this->Get(i,     j + 1) +
						// this->Get(i + 1, j - 1) +
						// this->Get(i - 1, j - 1) +
						this->Get(i    , j - 1)
					) / 4;

					tempGrid->Set_PF(i, j, meanNeighs);
				}
			}
		}

		//copia grade gerada para a grade principal
		for(int i = -OG_SEC_W; i < OG_SEC_W; i++)
			for(int j = -OG_SEC_H; j < OG_SEC_H; j++)
				this->Set_PF(i, j, tempGrid->Get(i, j));
		
	}

	this->ToStringPF();

	ROS_INFO("PF Finished.");

}

void OccupancyGrid::ToStringPF(){

	std::ofstream of;
	std::stringstream ss;
	std::string filename = "/home/darlan/Darlan/potentialfields.html";

	ss << "<html>\n"
		<< "<head>\n"
		<< "<meta http-equiv=\"refresh\" content=\"2\" />\n"
		<< "</head>\n"
	   	<< "<body style='background:black;'>\n"
	   	<< "<div style='text-align:center;margin:auto;'>\n"
		<< "<h3 style='color:white;font-family:sans'>Occupancy Grid</h3><br />\n"
		<< "<svg style='width:" << OG_WIDTH << "px; height:" << OG_HEIGHT << "px; display:block; margin:auto'>\n";

	int w = 0; 
	int h = 0;

	//from the maximum Y to the minimum Y, do
	for(int y = OG_SEC_H; y > -OG_SEC_H; y--){

		//from the minimum X to the maximum X, do
		for(int x = -OG_SEC_W; x < OG_SEC_W; x++){

			//convert cell values to a grayscale color
			double 	gfcolor 	= this->Get(x, -y);
			uint32_t gscolor 	= 255*255*255 * this->Get(x, -y);
			
			//ignore cell without any value
			if (gfcolor == 1){
				
				//add a pixel with the generated color to the bytestream
				ss << std::dec << "<rect x='" << (x + OG_SEC_W) << "' y='" << (y + OG_SEC_H) << "' width=1 height=1 ";
				ss << " fill='yellow' /> ";

			} else if (gfcolor != 0){

					//add a pixel with the generated color to the bytestream
					ss << std::dec << "<rect x='" << (x + OG_SEC_W) << "' y='" << (y + OG_SEC_H) << "' width=1 height=1 ";
					ss << " fill='#" << std::hex << std::setw(6) << std::setfill('0') << gscolor << "' /> ";

			}

			w++;
		}

		h++;
		w = 0;
	}


	ss << "<line x1='" << std:: dec << 0 << "' y1='" << OG_HEIGHT/2 << "' x2='" << OG_WIDTH << "' y2='" << OG_HEIGHT/2 << "' stroke=red />";
	ss << "<line x1='" << OG_WIDTH/2  << "' y1='0' x2='" << OG_WIDTH/2 << "' y2='" << OG_HEIGHT << "' stroke=red />";


	ss << "</svg></div>"
		<< "</body>"
		<< "</html>";

	of.open(filename.c_str(), std::ofstream::trunc);
	of << ss.str();
	of.close();			

}
