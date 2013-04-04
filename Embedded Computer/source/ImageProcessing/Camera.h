#ifndef CAMERA_H
#define CAMERA_H

#include <opencv\highgui.h>
#include "ImageProcessing.h"

using namespace cv;

/** @addtogroup Image Processing
 * @{
 */

/** \brief
 */
class Camera
{
public:
	enum class ColorSpace {RGB, HSV}; /**<  */

	static Camera& getInstance();
	bool initialize(int deviceIndex);
	void captureFrame();
	const Mat& getFrame(ColorSpace colorSpace);

private:
	Mat _rgbFrame; /**< RGB frame captured from the camera */
	Mat _hsvFrame; /**< HSV frame converted from the RGB frame */
	VideoCapture _capture;

	Camera() {}
	Camera(const Camera&) {}
	~Camera() {}

	void processFrame();
};

#endif // CAMERA_H
