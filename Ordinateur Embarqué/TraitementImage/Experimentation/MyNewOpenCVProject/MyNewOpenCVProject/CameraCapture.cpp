#include "CameraCapture.h"

using namespace cv;

CameraCapture::CameraCapture()
{
	CvSize frameSize = cvSize(WIDTH, HEIGHT);
	_rgbFrame = cvCreateImage(frameSize, IPL_DEPTH_8U, 3);
	_hsvFrame = cvCreateImage(frameSize, IPL_DEPTH_8U, 3);

	_capture = cvCaptureFromCAM(DEVICE_INDEX);
	if (!_capture)
	{
		// todo : réagir à l'erreur
	}
}

CameraCapture::~CameraCapture()
{
	cvReleaseCapture(&_capture);
}

CameraCapture& CameraCapture::getInstance()
{
	static CameraCapture instance;
	return instance;
}

void CameraCapture::processFrame()
{
	ImageProcessing::RGBtoHSV(_rgbFrame, _hsvFrame);
}

void CameraCapture::captureFrame()
{
	_rgbFrame = cvQueryFrame(_capture);
	processFrame();
}

IplImage* CameraCapture::getFrame(ColorSpace colorSpace)
{
	switch (colorSpace)
	{
	case ColorSpace::RGB:
		return _rgbFrame;
		break;
	case ColorSpace::HSV:
		return _hsvFrame;
		break;
	default:
		return _hsvFrame;
		break;
	}
}