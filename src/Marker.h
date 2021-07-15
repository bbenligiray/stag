#ifndef MARKER_H
#define MARKER_H

#include "Quad.h"

namespace stag {

class Marker : public Quad 
{
public:
	int id;
	cv::Mat C;

	Marker(const Quad &q, int inId);
	Marker(const Marker &m);
	void shiftCorners2(int shift);
};

} // namespace stag

#endif
