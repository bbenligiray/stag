#ifndef COMMONCLASSES_H
#define COMMONCLASSES_H

#define HALF_PI 1.570796326794897

#include <cmath>
#include "ED/EDLines.h"
#include "opencv2/opencv.hpp"

using cv::Point2d;
using cv::Point2i;
using cv::Point3d;
using cv::Mat;

class Corner
{
public:
	Point2d loc;
	LineSegment l1, l2;
	Corner(Point2d inLoc, LineSegment inL1, LineSegment inL2)
	{
		loc = inLoc;
		l1 = inL1;
		l2 = inL2;
	}
	Corner(){}
};



// checks if four corners form a quad, orders them clockwise
bool checkFourCorners(Corner& c1, Corner& c2, Corner& c3, Corner& c4, double thresManhDist);

// checks if two corners face each other, i.e., if they are in each other's fan
bool checkIfTwoCornersAreOpposite(const Corner& c1, const Corner& c2);

// checks if four corners face each other, assuming that c1 and c3 are facing each other
bool checkIfFourCornersAreOpposite(const Corner& c1, const Corner& c2, const Corner& c3, const Corner& c4);



// finds the intersection of two line segments
Point2d intersOfLineSegments(const LineSegment& line1, const LineSegment& line2);

// converts polar to Cartesien, scales it according to outer circle radius, shifts it to upper left corner
// the output is Mat, because we will be multiplying it with a homography matrix
Mat createMatFromPolarCoords(double radius, double radians, double circleRadius);

// calculates the cross product of two points
double crosProd(const Point2d& p1, const Point2d& p2);

// calculates the Manhattan distance between two points
double manhDistOfTwoPoints(const Point2d& p1, const Point2d& p2);

Point2d projectPoint(Point2d p, Mat H);

#endif