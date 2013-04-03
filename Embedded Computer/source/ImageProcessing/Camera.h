#ifndef CAMERA_H
#define CAMERA_H

#include <opencv\highgui.h>
#include "ImageProcessing.h"

class Camera
{
public:
	enum class ColorSpace {RGB, HSV};

	static Camera& getInstance();
	bool initialize(int deviceIndex);
	void captureFrame();
	const IplImage* getFrame(ColorSpace colorSpace);

private:
	const IplImage* _rgbFrame;
	IplImage* _hsvFrame;
	CvCapture* _capture;

	Camera() : _rgbFrame(nullptr), _hsvFrame(nullptr) {}
	Camera(const Camera&) {}
	~Camera();

	void processFrame();
};

#endif // CAMERA_H
