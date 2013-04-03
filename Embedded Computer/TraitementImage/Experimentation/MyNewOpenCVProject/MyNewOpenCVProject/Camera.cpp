#include "Camera.h"

using namespace cv;

Camera& Camera::getInstance()
{
	static Camera instance;
	return instance;
}

bool Camera::initialize()
{
	_capture = cvCaptureFromCAM(DEVICE_INDEX);
	return _capture;
}

void Camera::captureFrame()
{
	if (!_capture) {return;}

	_rgbFrame = cvQueryFrame(_capture);
	processFrame();
}

IplImage* Camera::getFrame(ColorSpace colorSpace)
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

Camera::~Camera()
{
	if (_capture) {cvReleaseCapture(&_capture);}
}

void Camera::processFrame()
{
	if (!_rgbFrame) {return;}
	
	CvSize frameSize = cvSize(_rgbFrame->width, _rgbFrame->height);
	_hsvFrame = cvCreateImage(frameSize, IPL_DEPTH_8U, 3);

	ImageProcessing::RGBtoHSV(_rgbFrame, _hsvFrame);
}