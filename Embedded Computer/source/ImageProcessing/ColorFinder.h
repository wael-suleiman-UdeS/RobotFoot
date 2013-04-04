#ifndef COLORFINDER_H
#define COLORFINDER_H

#include <cstdint>
#include "ImageProcessing.h"

using std::uint8_t;
using namespace cv;

struct HSVcolor
{
	uint8_t hue;
	uint8_t hueTolerance;
	uint8_t saturation;
	uint8_t brightness;
};

struct ShapeSpec
{
	int erosionIterations;
	int dilationIterations;
	int smoothingApertureSize;
	double resolutionDivisor;
	double minDistance;
};

struct CircleSpec : public ShapeSpec
{
	double edgeThreshold;
	double centerThreshold;
	double minRadius;
	double maxRadius;
};

class ColorFinder
{
public:
	ColorFinder(const HSVcolor* color);
	~ColorFinder() {}

	CvPoint getCirclePosition(const Mat& frame, const CircleSpec spec);

private:
	const HSVcolor* _color;
	Mat _resultFrame;

	ColorFinder() {};
	void setColor(const HSVcolor* color);
	void filter(const Mat& sourceFrame);
};

#endif // COLORFINDER_H
