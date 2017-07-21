#ifndef MARKER_H
#define MARKER_H

#include <vector>
#include "CommonClasses.h"
#include "opencv2/opencv.hpp"

using std::vector;


class Marker
{
	// calculates vanishing line
	void calcVanLine();

public:
	vector<Point2d> corners;
	Point2d center;
	Mat H;
	Point3d vanLine;
	Mat C;

	int id;
	double projDist = 0;

	Marker(){}
	Marker(Point2d c1, Point2d c2, Point2d c3, Point2d c4);



	// estimate H using corners and vanLine
	void estimatePose();

	// finds the projection of a marker point on the image. mainly used for decoding.
	Point2d projectPoint(const Mat &originalPoint);

	// reorder corners after decoding
	void shiftCorners(int shift);

	// calculates projective distortion using corners and vanishing line
	double calcProjDist();

	// checks if a point is inside a quad
	bool checkIfPointInQuad(const Point2d& p);
};
#endif