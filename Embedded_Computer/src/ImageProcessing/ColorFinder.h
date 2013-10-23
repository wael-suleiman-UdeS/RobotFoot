#ifndef COLORFINDER_H
#define COLORFINDER_H

#include <cstdint>
#include <boost/filesystem.hpp>
#include "ImageProcessing.h"
#include "../Utilities/XmlParser.h"

/** @addtogroup Image Processing
 * @{
 */

/** \brief Structure representing an HSV color
 */
struct HSVcolor
{
public:
	std::uint8_t hue;
	std::uint8_t hueTolerance;
	std::uint8_t saturation;
	std::uint8_t brightness;

	HSVcolor(const XmlParser& config, string colorName);
};

/** \brief Structure containing the specifications of a shape to find
 */
struct ShapeSpec
{
public:
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
public:
	double edgeThreshold;
	double centerThreshold;
	double minRadius;
	double maxRadius;

	CircleSpec(const XmlParser& config, string colorName);
};

/** \brief Class for finding a specified shape of a specified color in images
 */
class ColorFinder
{
public:
	ColorFinder(const HSVcolor* color);
	~ColorFinder() {}

	cv::Point getCirclePosition(const cv::Mat& frame, std::shared_ptr<CircleSpec> spec);

private:
	const HSVcolor* _color; /**< Color to find */
	cv::Mat _resultFrame; /**< Processed frame used to find the position of a shape */

	ColorFinder() {};
	void setColor(const HSVcolor* color);
	void filter(const cv::Mat& sourceFrame);
};

#endif // COLORFINDER_H
