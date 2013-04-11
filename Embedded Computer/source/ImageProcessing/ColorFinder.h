#ifndef COLORFINDER_H
#define COLORFINDER_H

#include <cstdint>
#include "ImageProcessing.h"

using std::uint8_t;
using namespace cv;

/** @addtogroup Image Processing
 * @{
 */

/** \brief Structure representing an HSV color
 */
struct HSVcolor
{
	uint8_t hue;
	uint8_t hueTolerance;
	uint8_t saturation;
	uint8_t brightness;
};

/** \brief Structure containing the specifications of a shape to find
 */
struct ShapeSpec
{
	int erosionIterations;
	int dilationIterations;
	int smoothingApertureSize;
	double resolutionDivisor;
	double minDistance;
};

/** \brief Structure containing the specifications of a circle to find
 */
struct CircleSpec : public ShapeSpec
{
	double edgeThreshold;
	double centerThreshold;
	double minRadius;
	double maxRadius;
};

/** \brief Class for finding a specified shape of a specified color in images
 */
class ColorFinder
{
public:
	ColorFinder(const HSVcolor* color);
	~ColorFinder() {}

	CvPoint getCirclePosition(const Mat& frame, const CircleSpec spec);

private:
	const HSVcolor* _color; /**< Color to find */
	Mat _resultFrame; /**< Processed frame used to find the position of a shape */

	ColorFinder() {};
	void setColor(const HSVcolor* color);
	void filter(const Mat& sourceFrame);
};

#endif // COLORFINDER_H
