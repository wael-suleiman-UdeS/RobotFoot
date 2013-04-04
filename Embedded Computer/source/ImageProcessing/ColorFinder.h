#ifndef COLORFINDER_H
#define COLORFINDER_H

#include <cstdint>
#include "ImageProcessing.h"

using std::uint8_t;

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

	CvPoint getCirclePosition(const IplImage* frame, const CircleSpec spec);

private:
	const HSVcolor* _color;
	IplImage* _resultFrame;

	ColorFinder() {};
	void setColor(const HSVcolor* color);
	void filter(const IplImage* frame);
};

#endif // COLORFINDER_H
