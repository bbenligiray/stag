#include "Marker.h"

void Marker::calcVanLine()
{
	// intersection points at the vanishing line
	Point2d inters1, inters2;

	// cross products of corners (i.e. lines representing the edges)
	double cross14 = crosProd(corners[0], corners[3]);
	double cross23 = crosProd(corners[1], corners[2]);
	double cross12 = crosProd(corners[0], corners[1]);
	double cross34 = crosProd(corners[2], corners[3]);

	// vectors going from one corner to another
	Point2d vec23(corners[1].x - corners[2].x, corners[1].y - corners[2].y);
	Point2d vec14(corners[0].x - corners[3].x, corners[0].y - corners[3].y);
	Point2d vec34(corners[2].x - corners[3].x, corners[2].y - corners[3].y);
	Point2d vec12(corners[0].x - corners[1].x, corners[0].y - corners[1].y);

	// if both edge pairs are parallel
	if ((crosProd(vec14, vec23) == 0) && (crosProd(vec12, vec34) == 0)) // lines are parallel
	{
		vanLine = Point3d(0, 0, 1);
		return;
	}
	// if one edge pair is parallel
	else if (crosProd(vec14, vec23) == 0)
	{
		inters2.x = (cross12 * vec34.x - vec12.x * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);
		inters2.y = (cross12 * vec34.y - vec12.y * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);

		//this intersection is not real. doing this to find the equation of the line with only one point.
		inters1.x = inters2.x + vec14.x;
		inters1.y = inters2.y + vec14.y;
	}
	// if the other edge pair is parallel
	else if (crosProd(vec12, vec34) == 0)
	{
		inters1.x = (cross14 * vec23.x - vec14.x * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);
		inters1.y = (cross14 * vec23.y - vec14.y * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);

		//this intersection is not real. doing this to find the equation of the line with only one point.
		inters2.x = inters1.x + vec12.x;
		inters2.y = inters1.y + vec12.y;
	}
	// if neither pairs are parallel
	else
	{
		inters1.x = (cross14 * vec23.x - vec14.x * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);
		inters1.y = (cross14 * vec23.y - vec14.y * cross23) / (vec14.x * vec23.y - vec14.y * vec23.x);

		inters2.x = (cross12 * vec34.x - vec12.x * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);
		inters2.y = (cross12 * vec34.y - vec12.y * cross34) / (vec12.x * vec34.y - vec12.y * vec34.x);
	}

	// find the vanishing line in homogeneous coordinates
	// l = P1 x P2
	double l1 = inters1.y - inters2.y;
	double l2 = inters2.x - inters1.x;
	double l3 = inters1.x * inters2.y - inters2.x * inters1.y;

	// normalize using http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/BEARDSLEY/node2.html (13)
	double normalizer = sqrt(l1 * l1 + l2 * l2);
	l1 /= normalizer;
	l2 /= normalizer;
	l3 /= normalizer;

	vanLine = Point3d(l1, l2, l3);
}



Marker::Marker(Point2d c1, Point2d c2, Point2d c3, Point2d c4)
{
	corners = vector<Point2d>(4);
	corners[0] = c1;
	corners[1] = c2;
	corners[2] = c3;
	corners[3] = c4;

	// this will either be needed estimate pose or projective distortion, might as well calculate it now
	calcVanLine();
}



void Marker::estimatePose()
{
	// apply affine rectification to the corners
	vector<Point2d> affineCorners = vector<Point2d>(4);

	for (int i = 0; i < 4; i++)
		affineCorners[i] = Point2d(corners[i].x / (vanLine.x * corners[i].x + vanLine.y * corners[i].y + vanLine.z), corners[i].y / (vanLine.x * corners[i].x + vanLine.y * corners[i].y + vanLine.z));

	Mat HarInv = Mat::eye(3, 3, CV_64FC1);
	Mat Haffsim = Mat::eye(3, 3, CV_64FC1);

	// inverse of affine rectification
	HarInv.at<double>(2, 0) = -vanLine.x / vanLine.z;
	HarInv.at<double>(2, 1) = -vanLine.y / vanLine.z;
	HarInv.at<double>(2, 2) = 1 / vanLine.z;

	// find the affine transformation from square to affine rectified quad
	Haffsim.at<double>(0, 0) = affineCorners[1].x - affineCorners[0].x;
	Haffsim.at<double>(0, 1) = affineCorners[3].x - affineCorners[0].x;
	Haffsim.at<double>(0, 2) = affineCorners[0].x;
	Haffsim.at<double>(1, 0) = affineCorners[1].y - affineCorners[0].y;
	Haffsim.at<double>(1, 1) = affineCorners[3].y - affineCorners[0].y;
	Haffsim.at<double>(1, 2) = affineCorners[0].y;

	// product of these transformations is the homography
	H = HarInv * Haffsim;

	// locate the projection of the center of the marker
	Mat origCenter(3, 1, CV_64FC1);
	origCenter.at<double>(0) = 0.5;
	origCenter.at<double>(1) = 0.5;
	origCenter.at<double>(2) = 1;

	origCenter = H * origCenter;
	center.x = origCenter.at<double>(0) / origCenter.at<double>(2);
	center.y = origCenter.at<double>(1) / origCenter.at<double>(2);
}

Point2d Marker::projectPoint(const Mat &originalPoint)
{
	Mat projectedPoint = H * originalPoint;
	return Point2d(projectedPoint.at<double>(0) / projectedPoint.at<double>(2), projectedPoint.at<double>(1) / projectedPoint.at<double>(2));
}

void Marker::shiftCorners(int shift)
{
	if (shift == 1)
	{
		Point2d t = corners[0];
		corners[0] = corners[1];
		corners[1] = corners[2];
		corners[2] = corners[3];
		corners[3] = t;
	}
	else if (shift == 2)
	{
		Point2d t1 = corners[0];
		Point2d t2 = corners[1];
		corners[0] = corners[2];
		corners[1] = corners[3];
		corners[2] = t1;
		corners[3] = t2;
	}
	else if (shift == 3)
	{
		Point2d t = corners[0];
		corners[0] = corners[3];
		corners[3] = corners[2];
		corners[2] = corners[1];
		corners[1] = t;
	}
	else
		return;

	// have to recalculate homography after shift
	estimatePose();
}

double Marker::calcProjDist()
{
	// find the minimum and maximum distance from corners to the vanishing line
	// projective distortion = maxDist / minDist
	double curDist = abs(vanLine.x * corners[0].x + vanLine.y * corners[0].y + vanLine.z);

	double minDist = curDist;
	double maxDist = curDist;

	for (int i = 1; i < 4; i++)
	{
		curDist = abs(vanLine.x * corners[i].x + vanLine.y * corners[i].y + vanLine.z);
		if (curDist < minDist)
			minDist = curDist;
		if (curDist > maxDist)
			maxDist = curDist;
	}
	projDist = maxDist / minDist;

	return projDist;
}

bool Marker::checkIfPointInQuad(const Point2d& p)
{
	// check if point p is in the fan of each corner
	Point2d c1c2, c1c4, c3c2, c3c4, c1p, c3p;

	c1c2 = Point2d(corners[1].x - corners[0].x, corners[1].y - corners[0].y);
	c1c4 = Point2d(corners[3].x - corners[0].x, corners[3].y - corners[0].y);
	c3c2 = Point2d(corners[1].x - corners[2].x, corners[1].y - corners[2].y);
	c3c4 = Point2d(corners[3].x - corners[2].x, corners[3].y - corners[2].y);

	c1p = Point2d(p.x - corners[0].x, p.y - corners[0].y);
	c3p = Point2d(p.x - corners[2].x, p.y - corners[2].y);

	if (crosProd(c1p, c1c2) * crosProd(c1p, c1c4) >= 0)
		return false;
	if (crosProd(c1c2, c1p) * crosProd(c1c2, c1c4) <= 0)
		return false;

	if (crosProd(c3p, c3c2) * crosProd(c3p, c3c4) >= 0)
		return false;
	if (crosProd(c3c2, c3p) * crosProd(c3c2, c3c4) <= 0)
		return false;

	return true;
}