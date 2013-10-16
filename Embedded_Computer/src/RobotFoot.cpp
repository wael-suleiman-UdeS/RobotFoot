/*!
 * \file RobotFoot.cpp
 * \brief The main file of the project
 * \authors Mitchel Labonté and Mickael Paradis
 * \version 0.2
 */
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/chrono.hpp>
#include <boost/uuid/uuid.hpp>

#include "ImageProcessing/Camera.h"
#include "ImageProcessing/ColorFinder.h"
#include "ImageProcessing/ObjectTracker.h"
#include "Utilities/XmlParser.h"
#include "Utilities/logger.h"
#include "Utilities/SerialInterface.h"
#include "Utilities/ThreadManager.h"
#include "Control/STM32F4.h"

/*!
 * \brief Track the ball
 *
 * A test function for the tracking of the ball
 *
 * \param mc : An instance of the micro controller
 * \param debug : Acivate some debug option
 * \param PID : Activate the use of PID
 * \param colorName : The string of the color of the ball
 */
void testTracking(STM32F4& mc, bool debug, bool PID, string colorName)
{
	Logger::getInstance().addStream(std::cout);
	Logger::getInstance() << "Initializing USB interface..." << std::endl;

	Logger::getInstance() << "Starting tracking demo." << std::endl;

	// Load config
	Logger::getInstance() << "Loading configuration file..." << std::endl;
	XmlParser config;
	if (!config.loadFile("config/config.xml"))
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

	double durationMean = 0;
	int durationIndex = 0;

	cv::Point ballPosition;
	HSVcolor color(config, colorName);
	CircleSpec circle(config, colorName);
	ColorFinder finder(&color);
	ObjectTracker tracker(&mc, Camera::getInstance().getCenter());
	tracker.initializeHack(config); // todo: holy hack
	if (PID)
	{
		tracker.initializeHackPID(config);
	}

	if (debug)
	{
		cv::namedWindow("BGR", CV_WINDOW_AUTOSIZE);
		cv::namedWindow("HSV", CV_WINDOW_AUTOSIZE);
	}

	Logger::getInstance() << "Tracking process started" << std::endl;
	while(true)
	{
      boost::chrono::system_clock::time_point start = boost::chrono::system_clock::now();
		Camera::getInstance().captureFrame();

		ballPosition = finder.getCirclePosition(Camera::getInstance().getFrame(Camera::ColorSpace::HSV),
			circle);

		//Logger::getInstance() << "Ball position: " << ballPosition.x << ", " << ballPosition.y << std::endl;

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

		if (!PID)
		{
			tracker.track(ballPosition);
		}
		else
		{
			tracker.trackPID(ballPosition);
		}

		if((cvWaitKey(10) & 255) == 27) break;
      boost::chrono::duration<double> sec = boost::chrono::system_clock::now() - start;
      durationMean += sec.count();
      durationIndex++;
      if(!(durationIndex <= 99)) 
      {
      	  std::cout << "took " << durationMean/100 << " seconds\n";
	  durationMean = 0;
	  durationIndex = 0;
      }
	}

	cvDestroyAllWindows();
}

/*!
 * \brief Use for the demo
 *
 *  Track the ball and use feedback for controlling the motor of the head
 *
 *  \param mc : An instance of the micro controller
 */
void hardSet(STM32F4& mc)
{
	using boost::filesystem::path;

	Logger::getInstance().addStream(std::cout);
	Logger::getInstance() << "Starting tracking demo." << std::endl;

	// Load config
	Logger::getInstance() << "Loading configuration file..." << std::endl;
	XmlParser config;
	if (!config.loadFile("config.xml"))
	{
		Logger::getInstance() << "Error while loading configuration file." << std::endl;
		return;
	}
	path basePath = XmlPath::Root / XmlPath::Motion / XmlPath::Motors / XmlPath::Head;

	int pan = config.getIntValue(basePath / XmlPath::Pan);
	int tilt = config.getIntValue(basePath / XmlPath::Tilt);

	mc.setTorque(pan, STM32F4::TorqueOn);
	mc.setTorque(tilt, STM32F4::TorqueOn);

	Logger::getInstance() << "Force set process started" << std::endl;
	int m;
	while(true)
	{
		Logger::getInstance() << "Set pan :" << std::endl;
		std::cin >> m;
		mc.setMotor(pan, m);

		Logger::getInstance() << "Set tilt :" << std::endl;
		std::cin >> m;
		mc.setMotor(tilt, m);

		if((cvWaitKey(10) & 255) == 27) break;
	}
}

int main_new(int argc, char * argv[])
{
   try
   {  
      // Add io stream to Logger
      Logger::getInstance().addStream(std::cout);

      // Load config file
      Logger::getInstance() << "Loading configuration file..." << std::endl;
      XmlParser config;
      if (!config.loadFile("config/config.xml")) 
      {
          Logger::getInstance(Logger::LogLvl::ERROR) << "Error while loading configuration file." << std::endl;
          std::exit(1);
      }

      // Set logging level  
      Logger::getInstance().setLogLvl(config.getStringValue(XmlPath::Root / "Logging" / "LogLvl"));

      // Thread Manager
      ThreadManager threadManager;

      // Init USB interface with STM32F4
      Logger::getInstance() << "Initializing USB interface..." << std::endl;
      boost::asio::io_service boost_io;
      std::string port_name = config.getStringValue(XmlPath::Root / "USB_Interface" / "TTY");
      STM32F4 mc(port_name, boost_io);
      threadManager.create(50, boost::bind(&boost::asio::io_service::run, &boost_io)); 
       
      // Start main process here
      Logger::getInstance() << "Done" << std::endl;
      boost_io.stop();
   }
   catch (std::exception& e)
   {
      Logger::getInstance(Logger::LogLvl::ERROR) << "Exception in main() : " << e.what() << std::endl;
   }
   return 0;
}

// Deprecated main
int main(int argc, char* argv[])
{

	try
	{
		boost::asio::io_service io;
		STM32F4 mc(argc > 1 ? std::string("/dev/") + argv[1] : std::string("/dev/ttyACM0"), io);
		boost::thread t(boost::bind(&boost::asio::io_service::run, &io));

		string color = "red";

		if (argc > 4)
		{
			color = argv[4];
		}

		if (argc > 3 && *argv[3] != 'b')
		{
			if (*argv[3] == 'h')
			{
				hardSet(mc);
			}
			else if (*argv[3] == 'p')
			{
				testTracking(mc, false, true, color);
			}
		}
		else
		{ 
			testTracking(mc, false, false, color);
		}

	}
	catch (std::exception& e)
    {
        std::cerr << "Exception :" << e.what() << std::endl;
    }
	// Initialize USB
	// TODO: handle USB exception
	//{
	//	boost::asio::io_service io_service;
	//	USBInterface usb(io_service, argc > 1 ? std::string("/dev/") + argv[1] : std::string("/dev/ttyUSB0"), 115200);
	//	boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service));

	//	testTracking(true, usb);
	//}
 //   catch (std::exception& e)
 //   {
 //       std::cerr << "Exception :" << e.what() << std::endl;
 //   }*/

	return 0;
}
