#ifndef POINT_COLMAP_H
#define POINT_COLMAP_H
#include <string>  
#include <vector>  
#include <sstream>  
// 3D point list with one line of data per point :
// POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as(IMAGE_ID, POINT2D_IDX)
struct Tracks {
	int image_id;
	int image_point2D_Index;
};
class Points3DColmap
{
private:
	int point3D_ID;
	float x, y, z, error;
	int R,G,B;
	std::vector<Tracks> tracks;
public:
	void writePointDate(int point_id, float x, float y, float z, int r, int g, int b, float error, std::vector<Tracks> track);
	std::string readOneLine();
	int getTranksCount();
 };


#endif // MAPDRAWER_H