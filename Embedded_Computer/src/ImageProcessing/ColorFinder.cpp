#include "ColorFinder.h"
#include "Utilities/logger.h"
#include <climits>
#include <boost/thread.hpp>

using boost::filesystem::path;
using cv::Mat;
using cv::vector;
using cv::Vec3f;
using cv::Scalar;
using cv::Point;

/** \brief Initialize the circle specifications from the XML configuration
 *
 * \param config XmlParser&: XML configuration containing the initialization informations
 * \param colorName string: Name of the HSV color
 * \return bool: Success of the initialization
 *
 */
HSVcolor::HSVcolor(const XmlParser& config, string colorName)
{
	path basePath = XmlPath::Root / XmlPath::ImageProcessing
		/ XmlPath::Colors / XmlPath::Color + "[@name=\"" + colorName + "\"]" / XmlPath::HSVcolor;

	hue = config.getIntValue(basePath / "Hue");
	hueTolerance = config.getIntValue(basePath / "HueTolerance");
	saturation = config.getIntValue(basePath / "Saturation");
	brightness = config.getIntValue(basePath / "Brightness");
}

/** \brief Initialigze the circle specifications from the XML configuration
 *
 * \param config XmlParser&: XML configuration containing the initialization informations
 * \param colorName string: Name of the color of the circle to find
 * \return bool: Success of the initialization
 *
 */
CircleSpec::CircleSpec(const XmlParser& config, string colorName)
{
	path basePath = XmlPath::Root / XmlPath::ImageProcessing
		/ XmlPath::Colors / XmlPath::Color + "[@name=\"" + colorName + "\"]" / XmlPath::CircleSpec;

	erosionIterations = config.getIntValue(basePath / "ErosionIterations");
	dilationIterations = config.getIntValue(basePath / "DilationIterations");
	smoothingApertureSize = config.getIntValue(basePath / "SmoothingApertureSize");
	resolutionDivisor = config.getIntValue(basePath / "ResolutionDivisor");
	minDistance = config.getIntValue(basePath / "MinDistance");

	edgeThreshold = config.getIntValue(basePath / "EdgeThreshold");
	centerThreshold = config.getIntValue(basePath / "CenterThreshold");
	minRadius = config.getIntValue(basePath / "MinRadius");
	maxRadius = config.getIntValue(basePath / "MaxRadius");

	//std::cout << "EROSION" << erosionIterations;
	//std::cout << "DILATION" << dilationIterations;
	//std::cout << "APERTURE" << smoothingApertureSize;
}

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
 * \return Point: Position of the first found circle
 *
 */
Point ColorFinder::getCirclePosition(const Mat& frame, std::shared_ptr<CircleSpec> spec)
{
	Point circlePosition = Point(-1, -1);

	if (frame.empty()) { return circlePosition; }

	filter(frame);
    
	ImageProcessing::erode(_resultFrame, _resultFrame, spec->erosionIterations);
	ImageProcessing::dilate(_resultFrame, _resultFrame, spec->dilationIterations);
    Logger::getInstance() << "test 3" << std::endl;
	ImageProcessing::smooth(_resultFrame, _resultFrame, spec->smoothingApertureSize);
    Logger::getInstance() << "test 4" << std::endl;
    
	boost::this_thread::sleep(boost::posix_time::millisec(10000));
	vector<Vec3f> circles;
	HoughCircles(_resultFrame, circles, CV_HOUGH_GRADIENT,
		spec->resolutionDivisor, spec->minDistance, spec->edgeThreshold,
		spec->centerThreshold, spec->minRadius, spec->maxRadius);

	if (circles.size())
	{
		circlePosition = Point(circles[0][0], circles[0][1]);
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
