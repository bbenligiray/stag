#include "opencv2/opencv.hpp"
#include "Stag.h"


void main()
{
	cv::Mat image = cv::imread("1.png", CV_LOAD_IMAGE_GRAYSCALE);

	Stag stag(15, 7, true);

	stag.detectMarkers(image);
	stag.logResults("log/");
}