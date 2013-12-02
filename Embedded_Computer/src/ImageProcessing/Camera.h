#ifndef CAMERA_H
#define CAMERA_H

#include <boost/filesystem.hpp>
#include <opencv/highgui.h>
#include "ImageProcessing.h"
#include "../Utilities/XmlParser.h"

/** @addtogroup Image Processing
 * @{
 */

/** \brief Singleton for image capturing from cameras
 */
class Camera
{
public:
	enum class ColorSpace {BGR, HSV}; /**< Enumeration representing color spaces */

	static Camera& getInstance();
	bool initialize(const XmlParser& config);
	bool initialize(int deviceIndex);
	void captureFrame();
	cv::Mat& getFrame(ColorSpace colorSpace);
	cv::Point getCenter();

private:
	cv::Mat _bgrFrame; /**< BGR frame captured from the camera */
	cv::Mat _hsvFrame; /**< HSV frame converted from the BGR frame */
	cv::VideoCapture _capture;	/**< Image capturing object */

	Camera() {}
	Camera(const Camera&) {}
	~Camera() {}

	void processFrame();
};

#endif // CAMERA_H
