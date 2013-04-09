#include "ColorFinder.h"
#include <climits>

/** \brief Constructor
 *
 * \param color const HSVcolor*: HSV Color to find
 *
 */
ColorFinder::ColorFinder(const HSVcolor* color)
{
	_color = color;
}

/** \brief Retrieve the position of a circle in the frame
 *
 * \param frame const Mat&: Frame in which to find the circle
 * \param spec CircleSpec: Specifications of the circle to find
 * \return CvPoint: Position of the first found circle
 *
 */
CvPoint ColorFinder::getCirclePosition(const Mat& frame, CircleSpec spec)
{
	CvPoint circlePosition = {-1, -1};

	if (frame.empty()) { return circlePosition; }

	filter(frame);

	ImageProcessing::erode(_resultFrame, _resultFrame, spec.erosionIterations);
	ImageProcessing::dilate(_resultFrame, _resultFrame, spec.dilationIterations);
	ImageProcessing::smooth(_resultFrame, _resultFrame, spec.smoothingApertureSize);
	
	vector<Vec3f> circles;
	HoughCircles(_resultFrame, circles, CV_HOUGH_GRADIENT,
		spec.resolutionDivisor, spec.minDistance, spec.edgeThreshold,
		spec.centerThreshold, spec.minRadius, spec.maxRadius);

	if (circles.size())
	{
		circlePosition.x = circles[0][0];
		circlePosition.y = circles[0][1];
	}

	return circlePosition;
}

/** \brief Set the color to find
 *
 * \param color const HSVcolor*: Color to find
 *
 */
void ColorFinder::setColor(const HSVcolor* color)
{
	_color = color;
}

/** \brief Filter a frame according to the setted HSV color
 *
 * \param sourceFrame const Mat&: Frame to filter
 *
 */
void ColorFinder::filter(const Mat& sourceFrame)
{
	if (sourceFrame.empty() || !_color) { return; }

	Scalar minHSV = cvScalar(_color->hue - _color->hueTolerance, _color->saturation, _color->brightness);
	Scalar maxHSV = cvScalar(_color->hue + _color->hueTolerance, UCHAR_MAX, UCHAR_MAX);

	ImageProcessing::filter(sourceFrame, _resultFrame, minHSV, maxHSV);
}
