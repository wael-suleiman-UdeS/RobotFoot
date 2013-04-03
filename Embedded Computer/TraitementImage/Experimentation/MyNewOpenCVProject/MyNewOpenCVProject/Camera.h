#pragma once

#include <opencv\highgui.h>
#include "ImageProcessing.h"

class Camera
{
public:
	enum class ColorSpace {RGB, HSV};

	static Camera& getInstance();
	bool initialize(int deviceIndex);
	void captureFrame();

	IplImage* getFrame(ColorSpace colorSpace);

private:
	IplImage* _rgbFrame;
	IplImage* _hsvFrame;
	CvCapture* _capture;

	Camera() {}
	Camera(const Camera&);
	~Camera();

	void processFrame();
};