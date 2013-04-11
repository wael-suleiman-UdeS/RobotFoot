#include "Camera.h"

/** \brief Retrieve the instance of the singleton
 *
 * \return Camera&: Instance of the singleton
 *
 */
Camera& Camera::getInstance()
{
	static Camera instance;
	return instance;
}

/** \brief Initialize the capture device
 *
 * \param deviceIndex int: Id of the camera to retrieve the frame from
 * \return bool: Success of the initialization
 *
 */
bool Camera::initialize(int deviceIndex)
{
	_capture = VideoCapture(deviceIndex);
	return _capture.isOpened();
}

/** \brief Retrieve a frame from the camera
 *
 */
void Camera::captureFrame()
{
	if (!_capture.isOpened()) { return; }

	_capture >> _bgrFrame;

	processFrame();
}

/** \brief Retrieve the captured frame in a specified color space
 *
 * \param colorSpace ColorSpace: Color space of the frame to get
 * \return const Mat&: Retrieved frame
 *
 */
const Mat& Camera::getFrame(Camera::ColorSpace colorSpace)
{
	switch (colorSpace)
	{
	case ColorSpace::BGR:
		return _bgrFrame;
	case ColorSpace::HSV:
	default:
		return _hsvFrame;
	}
}

/** \brief Retrieve the coordinates of the center of the frame captured by the camera
 *
 * \return CvPoint: Coordinates of the center of the camera image
 *
 */
CvPoint Camera::getCenter()
{
	CvPoint center = {-1, -1};
	if (!_capture.isOpened()) { return center; }

	center.x = _capture.get(CV_CAP_PROP_FRAME_WIDTH);
	center.y = _capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	return center;
}

/** \brief Convert the retrieved BRG frame in an HSV frame
 *
 */
void Camera::processFrame()
{
	if (!_bgrFrame.empty()) { return; }
	ImageProcessing::BGRtoHSV(_bgrFrame, _hsvFrame);
}
