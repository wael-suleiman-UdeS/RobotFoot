#include "Camera.h"

Camera& Camera::getInstance()
{
	static Camera instance;
	return instance;
}

/** \brief Initialize the capture device
 *
 * \param deviceIndex int
 * \return bool Success of the initializationc
 *
 */
bool Camera::initialize(int deviceIndex)
{
	_capture = VideoCapture(deviceIndex);
	return _capture.isOpened();
}

/** \brief Capture a frame
 *
 * \return void
 *
 */
void Camera::captureFrame()
{
	if (!_capture.isOpened()) { return; }

	_capture >> _rgbFrame;

	processFrame();
}

/** \brief Retrieve the captured frame
 *
 * \param colorSpace ColorSpace
 * \return const Mat
 *
 */
const Mat& Camera::getFrame(Camera::ColorSpace colorSpace)
{
	switch (colorSpace)
	{
	case ColorSpace::RGB:
		return _rgbFrame;
	case ColorSpace::HSV:
	default:
		return _hsvFrame;
	}
}

/** \brief Create an HSV frame from the captured RGB frame
 *
 * \return void
 *
 */
void Camera::processFrame()
{
	if (!_rgbFrame.empty()) { return; }
	ImageProcessing::RGBtoHSV(_rgbFrame, _hsvFrame);
}
