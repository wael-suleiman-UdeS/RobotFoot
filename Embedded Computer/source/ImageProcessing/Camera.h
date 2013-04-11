#ifndef CAMERA_H
#define CAMERA_H

#include <opencv\highgui.h>
#include "ImageProcessing.h"

using namespace cv;

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
	bool initialize(int deviceIndex);
	void captureFrame();
	const Mat& getFrame(ColorSpace colorSpace);
	CvPoint getCenter();

private:
	Mat _bgrFrame; /**< BGR frame captured from the camera */
	Mat _hsvFrame; /**< HSV frame converted from the BGR frame */
	VideoCapture _capture; /**< Image capturing object */

	Camera() {}
	Camera(const Camera&) {}
	~Camera() {}

	void processFrame();
};

#endif // CAMERA_H
