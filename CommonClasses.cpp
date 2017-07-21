#include "CommonClasses.h"

bool checkFourCorners(Corner& c1, Corner& c2, Corner& c3, Corner& c4, double thresManhDist)
{
	if (!checkIfTwoCornersAreOpposite(c1, c3))
		return false;

	// estimate c2 and c4 using c1 and c3
	Corner estC2, estC4;
	// there is two combinations when forming c2 and c4
	// we try the first one, check if it works. if not, use the second one.
	estC2 = Corner(intersOfLineSegments(c1.l1, c3.l1), c1.l1, c3.l1);
	estC4 = Corner(intersOfLineSegments(c1.l2, c3.l2), c1.l2, c3.l2);
	if (!checkIfFourCornersAreOpposite(c1, estC2, c3, estC4))
	{
		estC2 = Corner(intersOfLineSegments(c1.l1, c3.l2), c1.l1, c3.l2);
		estC4 = Corner(intersOfLineSegments(c1.l2, c3.l1), c1.l2, c3.l1);
	}

	// check the distances between detected corners and estimated corners
	// if they are close enough, detected corners are used.
	// using Manhattan distance because this is not critical at all, threshold is arbitrary anyway
	double distC2EstC2 = manhDistOfTwoPoints(c2.loc, estC2.loc);
	double distC2EstC4 = manhDistOfTwoPoints(c2.loc, estC4.loc);
	double distC4EstC2 = manhDistOfTwoPoints(c4.loc, estC2.loc);
	double distC4EstC4 = manhDistOfTwoPoints(c4.loc, estC4.loc);

	// at least on of c2 or c4 has to be correct
	if (distC2EstC2 < thresManhDist) // c2 is correct (estC2)
	{
		if (distC4EstC4 >= thresManhDist) // c4 is incorrect, replace with estC4
			c4 = estC4;
	}
	else if (distC2EstC4 < thresManhDist) // c2 is correct (estC4)
	{
		if (distC4EstC2 >= thresManhDist) // c4 is incorrect, replace with estC2
			c4 = estC2;
	}
	else // c2 is incorrect
	{
		if (distC4EstC2 < thresManhDist) // c4 is correct (estC2), replace c2 with estC4
			c2 = estC4;
		else if (distC4EstC4 < thresManhDist) // c4 is correct (estC4), replace c2 with estC2
			c2 = estC2;
		else // both are incorrect. no quad detection, because we need at least 3 corners
			return false;
	}

	// order corners in clockwise
	Point2d vec13(c3.loc.x - c1.loc.x, c3.loc.y - c1.loc.y);
	Point2d vec12(c2.loc.x - c1.loc.x, c2.loc.y - c1.loc.y);

	if (crosProd(vec13, vec12) > 0)
	{
		Corner temp = c2;
		c2 = c4;
		c4 = temp;
	}

	return true;
}

bool checkIfTwoCornersAreOpposite(const Corner& c1, const Corner& c2)
{
	// we need a corner intersection point, which comes from Corner.loc
	// we need two additional points, one from each line segment to define the corner
	// rather than using any point on the line segment, we choose the furthermost point from Corner.loc
	// we wouldn't need to do this if line segments were guaranteed to not intersect. however, there are some edge cases where this happens

	Point2d c1p1, c1p2, c2p1, c2p2, linePoint1, linePoint2;

	// choose a point for c1 from its line segment #1
	linePoint1 = Point2d(c1.l1.sx, c1.l1.sy);
	linePoint2 = Point2d(c1.l1.ex, c1.l1.ey);
	if (manhDistOfTwoPoints(c1.loc, linePoint1) > manhDistOfTwoPoints(c1.loc, linePoint2))
		c1p1 = Point2d(c1.l1.sx - c1.loc.x, c1.l1.sy - c1.loc.y);
	else
		c1p1 = Point2d(c1.l1.ex - c1.loc.x, c1.l1.ey - c1.loc.y);

	// choose a point for c1 from its line segment #2
	linePoint1 = Point2d(c1.l2.sx, c1.l2.sy);
	linePoint2 = Point2d(c1.l2.ex, c1.l2.ey);
	if (manhDistOfTwoPoints(c1.loc, linePoint1) > manhDistOfTwoPoints(c1.loc, linePoint2))
		c1p2 = Point2d(c1.l2.sx - c1.loc.x, c1.l2.sy - c1.loc.y);
	else
		c1p2 = Point2d(c1.l2.ex - c1.loc.x, c1.l2.ey - c1.loc.y);

	// choose a point for c2 from its line segment #1
	linePoint1 = Point2d(c2.l1.sx, c2.l1.sy);
	linePoint2 = Point2d(c2.l1.ex, c2.l1.ey);
	if (manhDistOfTwoPoints(c2.loc, linePoint1) > manhDistOfTwoPoints(c2.loc, linePoint2))
		c2p1 = Point2d(c2.l1.sx - c2.loc.x, c2.l1.sy - c2.loc.y);
	else
		c2p1 = Point2d(c2.l1.ex - c2.loc.x, c2.l1.ey - c2.loc.y);

	// choose a point for c2 from its line segment #2
	linePoint1 = Point2d(c2.l2.sx, c2.l2.sy);
	linePoint2 = Point2d(c2.l2.ex, c2.l2.ey);
	if (manhDistOfTwoPoints(c2.loc, linePoint1) > manhDistOfTwoPoints(c2.loc, linePoint2))
		c2p2 = Point2d(c2.l2.sx - c2.loc.x, c2.l2.sy - c2.loc.y);
	else
		c2p2 = Point2d(c2.l2.ex - c2.loc.x, c2.l2.ey - c2.loc.y);

	// create vectors from corner to corner
	Point2d c1c2(c2.loc.x - c1.loc.x, c2.loc.y - c1.loc.y);
	Point2d c2c1(c1.loc.x - c2.loc.x, c1.loc.y - c2.loc.y);

	// check if these two corners mutually have each other in their fan
	// is c2 inside c1's fan?
	if (crosProd(c1c2, c1p1) * crosProd(c1c2, c1p2) >= 0)
		return false;
	// is it behind it or in front of it?
	if (crosProd(c1p1, c1c2) * crosProd(c1p1, c1p2) <= 0)
		return false;

	// is c1 inside c2's fan?
	if (crosProd(c2c1, c2p1) * crosProd(c2c1, c2p2) >= 0)
		return false;
	// is it behind it or in front of it?
	if (crosProd(c2p1, c2c1) * crosProd(c2p1, c2p2) <= 0)
		return false;

	return true;
}

bool checkIfFourCornersAreOpposite(const Corner& c1, const Corner& c2, const Corner& c3, const Corner& c4)
{
	Point2d vec13(c3.loc.x - c1.loc.x, c3.loc.y - c1.loc.y);
	Point2d vec12(c2.loc.x - c1.loc.x, c2.loc.y - c1.loc.y);
	Point2d vec14(c4.loc.x - c1.loc.x, c4.loc.y - c1.loc.y);

	if (crosProd(vec13, vec12) * crosProd(vec13, vec14) >= 0)
		return false;

	Point2d vec24(c4.loc.x - c2.loc.x, c4.loc.y - c2.loc.y);
	Point2d vec21(c1.loc.x - c2.loc.x, c1.loc.y - c2.loc.y);
	Point2d vec23(c3.loc.x - c2.loc.x, c3.loc.y - c2.loc.y);

	if (crosProd(vec24, vec21) * crosProd(vec24, vec23) >= 0)
		return false;

	return true;
}



Point2d intersOfLineSegments(const LineSegment& line1, const LineSegment& line2)
{
	Point2d inters;

	double aL1, bL1, aL2, bL2;
	if (line1.invert == 0)
	{
		aL1 = line1.b;
		bL1 = line1.a;
	}
	else
	{
		aL1 = 1 / line1.b;
		bL1 = -line1.a / line1.b;
	}
	if (line2.invert == 0)
	{
		aL2 = line2.b;
		bL2 = line2.a;
	}
	else
	{
		aL2 = 1 / line2.b;
		bL2 = -line2.a / line2.b;
	}
	inters.x = (bL2 - bL1) / (aL1 - aL2);
	inters.y = aL1 * inters.x + bL1;

	if ((line1.invert == 1) && (line1.b == 0))
	{
		if (line2.invert == 0)
		{
			inters.y = line2.a + line2.b * line1.a;
			inters.x = line1.a;
		}
		else
		{
			inters.y = (line1.a - line2.a) / line2.b;
			inters.x = line1.a;
		}
	}
	else if ((line2.invert == 1) && (line2.b == 0))
	{
		if (line1.invert == 0)
		{
			inters.y = line1.a + line1.b * line2.a;
			inters.x = line2.a;
		}
		else
		{
			inters.y = (line2.a - line1.a) / line1.b;
			inters.x = line2.a;
		}
	}
	return inters;
}

Mat createMatFromPolarCoords(double radius, double radians, double circleRadius)
{
	Mat point(3, 1, CV_64FC1);
	point.at<double>(0) = 0.5 + cos(radians) * radius * (circleRadius / 0.5);
	point.at<double>(1) = 0.5 - sin(radians) * radius * (circleRadius / 0.5);
	point.at<double>(2) = 1;
	return point;
}

double crosProd(const Point2d& p1, const Point2d& p2)
{
	return p1.x * p2.y - p1.y * p2.x;
}

double manhDistOfTwoPoints(const Point2d& p1, const Point2d& p2)
{
	return abs(p1.x - p2.x) + abs(p1.y - p2.y);
}

Point2d projectPoint(Point2d p, Mat H)
{
	Mat pMat = (cv::Mat_<double>(3, 1) << p.x, p.y, 1);
	Mat projPMat = H * pMat;
	return Point2d(projPMat.at<double>(0) / projPMat.at<double>(2), projPMat.at<double>(1) / projPMat.at<double>(2));
}