#include <iostream>
#include <boost\asio.hpp>
#include <boost\thread.hpp>

#include "ImageProcessing\Camera.h"
#include "ImageProcessing\ColorFinder.h"
#include "ImageProcessing\ObjectTracker.h"
#include "Utilities\XmlParser.h"
#include "Utilities\logger.h"
#include "Utilities\USBInterface.h"

void testTracking(bool debug, USBInterface& usb)
{
	Logger::getInstance().addStream(std::cout);
	Logger::getInstance() << "Initializing USB interface..." << std::endl;

	Logger::getInstance() << "Starting tracking demo." << std::endl;

	// Load config
	Logger::getInstance() << "Loading configuration file..." << std::endl;
	XmlParser config;
	if (!config.loadFile("config.xml"))
	{
		Logger::getInstance() << "Error while loading configuration file." << std::endl;
		return;
	}

	// Initialize capture
	Logger::getInstance() << "Initializing capture device..." << std::endl;
	if (!Camera::getInstance().initialize(config))
	{
		Logger::getInstance() << "Error while initializing capture device." << std::endl;
		return;
	}

	cv::Point ballPosition;
	HSVcolor color(config, "red");
	CircleSpec circle(config, "red");
	ColorFinder finder(&color);
	ObjectTracker tracker(&usb, Camera::getInstance().getCenter());

	if (debug)
	{
		cv::namedWindow("BGR", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("HSV", CV_WINDOW_AUTOSIZE);
	}

	Logger::getInstance() << "Tracking process started" << std::endl;
	while(true)
	{
		Camera::getInstance().captureFrame();

		ballPosition = finder.getCirclePosition(Camera::getInstance().getFrame(Camera::ColorSpace::HSV),
			circle);

		if (debug)
		{
			if (ballPosition.x > -1 && ballPosition.y > -1)
			{
				cv::Scalar circleColor = cvScalar(255, 0, 0);
				cv::circle(Camera::getInstance().getFrame(Camera::ColorSpace::BGR), ballPosition, 5, circleColor);
			}

			cv::imshow("BGR", Camera::getInstance().getFrame(Camera::ColorSpace::BGR));
			cv::imshow("HSV", Camera::getInstance().getFrame(Camera::ColorSpace::HSV));
		}

		tracker.track(ballPosition);

		if((cvWaitKey(10) & 255) == 27) break;
	}

	cvDestroyAllWindows();
}

int main(int argc, char* argv[])
{
	// Initialize USB
	// TODO: handle USB exception
	try
	{
		boost::asio::io_service io_service;
		USBInterface usb(io_service, argc > 1 ? std::string("/dev/") + argv[1] : std::string("/dev/ttyUSB0"), 115200);
		boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service));

		testTracking(true, usb);
	}
    catch (std::exception& e)
    {
        std::cerr << "Exception :" << e.what() << std::endl;
    }

	return 0;
}