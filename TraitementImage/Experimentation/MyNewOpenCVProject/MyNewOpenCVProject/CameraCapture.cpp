#include "CameraCapture.h"

using namespace cv;

CameraCapture* CameraCapture::_instance = NULL;

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
	_instance = NULL;
}

CameraCapture* CameraCapture::GetInstance()
{
	if (!_instance) {_instance = new CameraCapture();}
	return _instance;
}

void CameraCapture::ProcessFrame()
{
	ImageProcessing::RGBtoHSV(_rgbFrame, _hsvFrame);
}

void CameraCapture::CaptureFrame()
{
	_rgbFrame = cvQueryFrame(_capture);
	ProcessFrame();
}

IplImage* CameraCapture::GetFrame(ColorSpace colorSpace)
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