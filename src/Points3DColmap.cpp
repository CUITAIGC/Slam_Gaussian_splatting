
#include "Points3DColmap.h"

void Points3DColmap::writePointDate(int point_id, float x, float y, float z, int r, int g, int b, float error, std::vector<Tracks> track)
{
	point3D_ID = point_id;
	this->x = x;
	this->y = y;
	this->z = z;
	this->R = r;
	this->G = g;
	this->B = b;
	this->error = error;
	this->tracks.clear();
	tracks = track;

}

std::string Points3DColmap::readOneLine()
{
	std::ostringstream oss;
	oss << point3D_ID<<" ";
	oss << x << " ";
	oss << y << " ";
	oss << z << " ";
	oss << R << " ";
	oss << G << " ";
	oss << B << " ";
	oss << error << " ";
	for (int i = 0; i < tracks.size(); i++) {
		oss << tracks[i].image_id<<" ";
		oss << tracks[i].image_point2D_Index << " ";
	}
	oss<< "\r";
	return oss.str();
}
int Points3DColmap::getTranksCount(){
	return tracks.size();
}
