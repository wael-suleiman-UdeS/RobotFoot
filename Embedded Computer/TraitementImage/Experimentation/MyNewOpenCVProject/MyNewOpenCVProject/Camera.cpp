#include "Camera.h"

using namespace cv;

Camera& Camera::getInstance()
{
	static Camera instance;
	return instance;
}

bool Camera::initialize()
{
	CvSize frameSize = cvSize(WIDTH, HEIGHT);
	_rgbFrame = cvCreateImage(frameSize, IPL_DEPTH_8U, 3);
	_hsvFrame = cvCreateImage(frameSize, IPL_DEPTH_8U, 3);

	_capture = cvCaptureFromCAM(DEVICE_INDEX);
	return !_capture;
}

void Camera::captureFrame()
{
	if (!_capture) {return;}

	_rgbFrame = cvQueryFrame(_capture);
	processFrame();
}

IplImage* Camera::getFrame(ColorSpace colorSpace)
{
	if (!_rgbFrame || !_hsvFrame) {return NULL;}

	switch (colorSpace)
	{
	case ColorSpace::RGB:
		return _rgbFrame;
		break;
	case ColorSpace::HSV:
	default:
		return _hsvFrame;
		break;
	}
}

Camera::~Camera()
{
	if (_capture) {cvReleaseCapture(&_capture);}
}

void Camera::processFrame()
{
	if (!_rgbFrame || !_hsvFrame
		&& !(_rgbFrame.width == _hsvFrame.width && _rgbFrame.height == _hsvFrame.height)) {return;}

	ImageProcessing::RGBtoHSV(_rgbFrame, _hsvFrame);
}