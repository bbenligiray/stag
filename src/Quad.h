#ifndef QUAD_H
#define QUAD_H

#include <vector>

#include "opencv2/opencv.hpp"

class Quad
{
public:
	std::vector<cv::Point2d> corners;
	cv::Point3d lineInf;
	double projectiveDistortion = 0;
	cv::Mat H;
	cv::Point2d center;

	void calculateLineAtInfinity();
	void calculateProjectiveDistortion();

	Quad(){}
	Quad(std::vector<cv::Point2d> inCorners);
	Quad(const Quad &q);
	void estimateHomography();
};

#endif