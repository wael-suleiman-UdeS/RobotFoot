#include "Control/MotorControl_2.h"
#include "Utilities/logger.h"

#include <iostream>
#include <iterator>
#include <algorithm>
#include <functional>
#include <boost/algorithm/clamp.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <cmath>

using boost::filesystem::path;

//#define DEBUG_TEST_MOTION

using boost::algorithm::clamp;


#define DANGER_TEST_MOTION

namespace
{
	const double dAngleConvertion = 0.325;
	const double dInvAngleConvertion = 1/dAngleConvertion;
}

Motor::Motor(STM32F4 *stm32f4, std::string name, int id, int offset, int min, int max, int playTime, bool isInversed)
:
_stm32f4(stm32f4),
_name(name),
_id(id),
_offset(offset),
_min(min),
_max(max),
_playTime(playTime),
_lastPos(0),
_currentPos(0),
_isInversed(isInversed)
{
}

Motor::~Motor()
{
}

void Motor::setPos(double pos)
{
    if(_isInversed)
    {
        _currentPos = -pos;
    }
    else
    {
        _currentPos = pos;
    }
}

const double Motor::getPos()
{
    if(_isInversed)
    {
        return -_currentPos;
    }
    else
    {
        return _currentPos;
    }
}

void Motor::setTorque(bool value)
{
    _stm32f4->setTorque(_id, value ? STM32F4::TorqueOn : STM32F4::TorqueOff);
}

int Motor::Angle2Value(const double angle)
{
    int value = (angle*dInvAngleConvertion) + _offset;
	if(value < _min || value > _max)
    {
        Logger::getInstance(Logger::LogLvl::DEBUG) << __FILE__ << " : Angle2Value : Value off limits.";
    }
    return clamp(value, _min, _max);
}

double Motor::Value2Angle(const int value)
{
	if(value < _min || value > _max)
    {
        Logger::getInstance(Logger::LogLvl::DEBUG) << __FILE__ << " : Angle2Value : Value off limits.";
    }
	int clampedValue = clamp(value, _min, _max);
	return double(clampedValue - _offset)*dAngleConvertion;
}

const double Motor::getMinAngle()
{
	return Value2Angle(_min);
}

const double Motor::getMaxAngle()
{
	return Value2Angle(_max);
}

void Motor::Read()
{
    std::int16_t value = _stm32f4->read(_id);
    if (value > 0)
        _currentPos = Value2Angle(value);
    else
    	Logger::getInstance(Logger::LogLvl::DEBUG) << __FILE__ << " : Read Error Id : " << _id << std::endl;
}

void Motor::Write()
{
    if (_currentPos != _lastPos)
    {
        _lastPos = _currentPos;
        _stm32f4->setMotor(_id, Angle2Value(_currentPos), _playTime);
    }
}

MotorControl::MotorControl(std::shared_ptr<ThreadManager> threadManager_ptr, const XmlParser &config, boost::asio::io_service &boost_io) :
 _threadManager(threadManager_ptr)
{
    try
    {
        // Init USB interface with STM32F4
        Logger::getInstance() << "Initializing USB interface..." << std::endl;

        std::string port_name = config.getStringValue(XmlPath::Root / "USB_Interface" / "TTY");

        _stm32f4 = new STM32F4(port_name, boost_io);
        _threadManager->create(90, boost::bind(&boost::asio::io_service::run, &boost_io));
        _robotHeight = config.getIntValue(XmlPath::Root / XmlPath::Sizes / "RobotHeight");
    }
    catch (std::exception& e)
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "Exception in MotorControl.cpp while initialising USB interface : " << e.what() << std::endl;
    }

    InitializeMotors(config);
    InitializeConfigurations(config);

}

MotorControl::~MotorControl()
{

}

void MotorControl::run()
{
    try
    {

        boost::chrono::system_clock::time_point start = boost::chrono::system_clock::now();
        while(1)
        {
            boost::chrono::duration<double> sec = boost::chrono::system_clock::now() - start;
            Logger::getInstance(Logger::LogLvl::DEBUG) << "ALL took " << sec.count() << " seconds" << std::endl;
            start = boost::chrono::system_clock::now();

            // Main task reading and sending data
            Logger::getInstance(Logger::LogLvl::DEBUG) << "MotorControl : ReadAll" << std::endl;
            ReadAll();
            if (_threadManager->resume(ThreadManager::Task::LEGS_CONTROL))
            {
                Logger::getInstance(Logger::LogLvl::DEBUG) << "MotorControl : wait for StaticWalk" << std::endl;
                _threadManager->wait();
            }
            else
            {
                Logger::getInstance(Logger::LogLvl::DEBUG) << "MotorControl : Resume LEGS_CONTROL fail" << std::endl;
            }

            Logger::getInstance(Logger::LogLvl::DEBUG) << "MotorControl : WriteAll" << std::endl;
            WriteAll();
                        _threadManager->wait(); 
        }
    }
    catch(boost::thread_interrupted const &e)
    {
        Logger::getInstance() << "MOTOR_CONTROL task Interrupted. " << std::endl;
    }  
}

// Populate the motor list
void MotorControl::InitializeMotors(const XmlParser &config)
{
    std::map<std::string, path> paths;
    paths.insert(std::make_pair("R_HIP_YAW", XmlPath::LegsMotors / XmlPath::R_HIP_YAW));
	paths.insert(std::make_pair("L_HIP_YAW", XmlPath::LegsMotors / XmlPath::L_HIP_YAW));
	paths.insert(std::make_pair("R_HIP_ROLL", XmlPath::LegsMotors / XmlPath::R_HIP_ROLL));
	paths.insert(std::make_pair("L_HIP_ROLL", XmlPath::LegsMotors / XmlPath::L_HIP_ROLL));
	paths.insert(std::make_pair("R_HIP_PITCH", XmlPath::LegsMotors / XmlPath::R_HIP_PITCH));
	paths.insert(std::make_pair("L_HIP_PITCH", XmlPath::LegsMotors / XmlPath::L_HIP_PITCH));
	paths.insert(std::make_pair("R_KNEE", XmlPath::LegsMotors / XmlPath::R_KNEE));
	paths.insert(std::make_pair("L_KNEE", XmlPath::LegsMotors / XmlPath::L_KNEE));
	paths.insert(std::make_pair("R_ANKLE_PITCH", XmlPath::LegsMotors / XmlPath::R_ANKLE_PITCH));
	paths.insert(std::make_pair("L_ANKLE_PITCH", XmlPath::LegsMotors / XmlPath::L_ANKLE_PITCH));
	paths.insert(std::make_pair("R_ANKLE_ROLL", XmlPath::LegsMotors / XmlPath::R_ANKLE_ROLL));
	paths.insert(std::make_pair("L_ANKLE_ROLL", XmlPath::LegsMotors / XmlPath::L_ANKLE_ROLL));
	paths.insert(std::make_pair("HEAD_PAN", XmlPath::HeadMotors / XmlPath::HEAD_PAN));
	paths.insert(std::make_pair("HEAD_TILT", XmlPath::HeadMotors / XmlPath::HEAD_TILT));

	for (auto it = paths.begin(); it != paths.end(); ++it)
	{
	    int id        = config.getIntValue(it->second / XmlPath::MotorID);
		int offset    = config.getIntValue(it->second / XmlPath::Offset);
		int min       = config.getIntValue(it->second / XmlPath::LimitMin);
		int max       = config.getIntValue(it->second / XmlPath::LimitMax);
        int playTime  = config.getIntValue(it->second / XmlPath::PlayTime);    
        bool isInversed = config.getIntValue(it->second / XmlPath::IsInversed);    
        Motor *motor = new Motor(_stm32f4, it->first, id, offset, min, max, playTime, isInversed);
	    _motors.insert(std::make_pair(it->first, motor));
    }
    
    InitPID(config);
}

void MotorControl::InitPID(const XmlParser &config)
{
    int KP1 = config.getIntValue(XmlPath::Root / XmlPath::Motion / XmlPath::KP);
    int KP2 = KP1 >> 8;
    int KD1 = config.getIntValue(XmlPath::Root / XmlPath::Motion / XmlPath::KD);
    int KD2 = KD1 >> 8;
    int KI1 = config.getIntValue(XmlPath::Root / XmlPath::Motion / XmlPath::KI);
    int KI2 = KI1 >> 8;

	for(int id = 1; id < 15; id++)
	{
		_stm32f4->writeRAM(id,24,KP1);
		_stm32f4->writeRAM(id,25,KP2);
		_stm32f4->writeRAM(id,26,KD1);
		_stm32f4->writeRAM(id,27,KD2);
		_stm32f4->writeRAM(id,28,KI1);
		_stm32f4->writeRAM(id,29,KI2);
	}
	_stm32f4->writeRAM(253,24,KP1);
	_stm32f4->writeRAM(253,25,KP2);
	_stm32f4->writeRAM(253,26,KD1);
	_stm32f4->writeRAM(253,27,KD2);
	_stm32f4->writeRAM(253,28,KI1);
	_stm32f4->writeRAM(253,29,KI2);
}

// Populate the configuration list
void MotorControl::InitializeConfigurations(const XmlParser &config)
{
    std::map<Config, path> paths;
    paths.insert(std::make_pair(Config::ALL_MOTORS, XmlPath::MotorsConfig / "ALL_MOTORS"));
    paths.insert(std::make_pair(Config::ALL_LEGS, XmlPath::MotorsConfig / "ALL_LEGS"));
    paths.insert(std::make_pair(Config::RIGHT_LEG, XmlPath::MotorsConfig / "RIGHT_LEG"));
    paths.insert(std::make_pair(Config::LEFT_LEG, XmlPath::MotorsConfig / "LEFT_LEG"));
    paths.insert(std::make_pair(Config::HEAD, XmlPath::MotorsConfig / "HEAD"));

    for (auto it = paths.begin(); it != paths.end(); ++it)
    {
        std::vector<Motor*> motors;
        std::vector<std::string> names = config.getChildrenStringValues(it->second);
        for (auto name = names.begin(); name != names.end(); ++name)
        {
            if (_motors.find(*name) != _motors.end())
                motors.push_back(_motors[*name]);
        } 
        _configurations.insert(std::make_pair(it->first, motors));
    }
}

bool MotorControl::SetTorque(bool value, const Config config)
{
   bool status = true;
   if (_configurations.find(config) == _configurations.end())
   {
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"SetTorque\" : Configuration is invalid." << std::endl;
       return false;
   }

   for (auto it = _configurations[config].begin(); it != _configurations[config].end() && status; ++it)
   {
       // TODO : Grab motor status
       (*it)->setTorque(value);
   }
   return status;
}

bool MotorControl::InitPositions(const std::vector<double>& desiredPos, const Config config,
				                const double msTotalTime /*= 10000.0*/,
				                const double msDt /*= 16*/ )
{
   if (_configurations.find(config) == _configurations.end())
   {
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"InitPosition\" : Configuration is invalid." << std::endl;
       return false;
   }
   
   if (desiredPos.size() != _configurations[config].size())
   {
#ifdef DEBUG_TEST_MOTION
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"InitPosition\" : Joint number is invalid." << std::endl;
#endif
       return false;
   }

   Logger::getInstance(Logger::LogLvl::INFO) << "Setting initial position" << std::endl;
#ifdef DEBUG_TEST_MOTION
   Logger::getInstance(Logger::LogLvl::DEBUG) << "";
   std::copy(desiredPos.begin(), desiredPos.end(), std::ostream_iterator<double>(std::cout, " "));
   Logger::getInstance(Logger::LogLvl::DEBUG) << std::endl;
#endif   

   ReadAll();

   std::vector<double> pos;
   if (!ReadPositions(pos, config))
   {
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"InitPosition\" : Error ready positions.";
       return false;
   }
   std::vector<double> posDt;

   auto itrDesiredPos = desiredPos.begin();
   auto endDesiredPos = desiredPos.end();
   auto itrPos = pos.begin();
   auto endPos = pos.end();

   // Calcul position dt for each motor
   for ( ; itrDesiredPos != endDesiredPos || itrPos != endPos; itrDesiredPos++, itrPos++ )
   {
      posDt.push_back( (*itrDesiredPos - *itrPos)/(msTotalTime/msDt) );
   }

   // Loop to add position dt to motor position and send command to motor
   for (double t = 0.0; t < msTotalTime; t+=msDt)
   {
      std::transform(pos.begin(), pos.end(), posDt.begin(), pos.begin(), std::plus<double>());

      SetPositions(pos, config);
      WriteAll();
      usleep(msDt*1000);
   }
   return true;
}

bool MotorControl::SetPosition(double pos, std::string name)
{  
    if (_motors.find(name) == _motors.end())
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"SetPosition\" : Motor " << name << " invalid." << std::endl;
        return false;
    }

    _motors[name]->setPos(pos);
    return true;  
}

bool MotorControl::SetPositions(const std::vector<double>& pos, const Config config)
{
   if (_configurations.find(config) == _configurations.end())
   {
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"SetPositions\" : Configuration is invalid." << std::endl;
       return false;
   }
   
   if (pos.size() != _configurations[config].size())
   {
#ifdef DEBUG_TEST_MOTION
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"SetPositions\" : Joint number is invalid." << std::endl;
#endif
       return false;
   }

   bool status = true;

#ifdef DANGER_TEST_MOTION   
   auto itrJoint = _configurations[config].begin();
   const auto endJoint = _configurations[config].end();
   auto itrPos = pos.begin();
   const auto endPos = pos.end();

   for ( ; itrJoint != endJoint && itrPos != endPos && status ; itrJoint++, itrPos++ )
   {
       (*itrJoint)->setPos(*itrPos);
   }
#endif

#ifdef DEBUG_TEST_MOTION
      Logger::getInstance(Logger::LogLvl::DEBUG) << "";
      std::copy(pos.begin(), pos.end(), std::ostream_iterator<double>(std::cout, " "));
      Logger::getInstance(Logger::LogLvl::DEBUG) << std::endl;
#endif
   return status;
}

void MotorControl::HardSet(const std::vector<double>& pos, const Config config)
{  
   boost::mutex::scoped_lock lock(_io_mutex);
   auto itrJoint = _configurations[config].begin();
   const auto endJoint = _configurations[config].end();
   auto itrPos = pos.begin();
   const auto endPos = pos.end();
   //Logger::getInstance() << " HardSet : ";
   for ( ; itrJoint != endJoint && itrPos != endPos; itrJoint++, itrPos++ )
   {
       (*itrJoint)->setPos(*itrPos);
       (*itrJoint)->Write();
       //Logger::getInstance() << *itrPos << " ";
   }
   //Logger::getInstance() << std::endl;
}

void MotorControl::HardGet(std::vector<double>& pos, const Config config)
{
   boost::mutex::scoped_lock lock(_io_mutex);
   if (_configurations.find(config) == _configurations.end())
   {
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"HardGet\" : Configuration is invalid." << std::endl;
       return;
   }
   auto itrJoint = _configurations[config].begin();
   const auto endJoint = _configurations[config].end();
   Logger::getInstance() << "HardGet : ";
   for ( ; itrJoint != endJoint; itrJoint++)
   {
       (*itrJoint)->Read();
       pos.push_back((*itrJoint)->getPos());
       Logger::getInstance() << (*itrJoint)->getPos() << " ";
   }
   Logger::getInstance() << std::endl;
}

void MotorControl::HardGetMaxAngles(std::vector<double>& angles, const Config config)
{
	if (_configurations.find(config) == _configurations.end())
	{
	   Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"HardGetMaxAngles\" : Configuration is invalid." << std::endl;
	   return;
    }
	auto itrJoint = _configurations[config].begin();
	const auto endJoint = _configurations[config].end();

    for ( ; itrJoint != endJoint; itrJoint++)
    {
    	angles.push_back((*itrJoint)->getMaxAngle());
    }
}

void MotorControl::HardGetMinAngles(std::vector<double>& angles, const Config config)
{
	if (_configurations.find(config) == _configurations.end())
	{
	   Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"HardGetMinAngles\" : Configuration is invalid." << std::endl;
	   return;
    }
	auto itrJoint = _configurations[config].begin();
	const auto endJoint = _configurations[config].end();

    for ( ; itrJoint != endJoint; itrJoint++)
    {
    	angles.push_back((*itrJoint)->getMinAngle());
    }
}

const double MotorControl::ReadPosition(std::string name)
{
    if (_motors.find(name) == _motors.end())
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"ReadPosition\" : Motor " << name << " invalid." << std::endl;
        return -1;
    }
    return _motors[name]->getPos();
}

bool MotorControl::ReadPositions(std::vector<double>& pos, const Config config)
{
   if (_configurations.find(config) == _configurations.end())
   {
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"ReadPositions\" : Configuration is invalid." << std::endl;
       return false;
   }

   bool status = true;
   auto itr = _configurations[config].begin();
   const auto end = _configurations[config].end();

   for ( ; itr != end && status ; itr++ )
   {
       pos.push_back((*itr)->getPos());
   }
   return status;
}

void MotorControl::ReadAll()
{
    for (auto it = _motors.begin(); it != _motors.end(); ++it)
    {
        it->second->Read();
    }
}

void MotorControl::WriteAll()
{
    for (auto it = _motors.begin(); it != _motors.end(); ++it)
    {
        it->second->Write();
    }
}

string MotorControl::GetColorToTrack()
{
	//string ballColor = _config.getStringValue(XmlPath::Root / XmlPath::ImageProcessing / XmlPath::Color / XmlPath::BallColor);
	//string goalColor = _config.getStringValue(XmlPath::Root / XmlPath::ImageProcessing / XmlPath::Color / XmlPath::GoalColor);

	return "red";
}

void MotorControl::TestCalculFun() {
	_goalPosition = GetObjectDistance();
	_ballPosition = GetObjectDistance();

	ObjectPosition distance;
	distance.x = _goalPosition.x - _ballPosition.x;
	distance.y = _goalPosition.y - _ballPosition.y;

	if (_ballPosition.x < 0) {
		// Fuck off
	}
	else if (_ballPosition.y < 0) {
		// 180 - std::atan(std::abs(distance.y / distance.x));
	}
	else {
		// 180 - std::atan(std::abs(distance.x / distance.y));
	}

}

ObjectPosition MotorControl::GetObjectDistance()
{

	// todo: replace hard gets

	std::vector<double> angles;
	//_mc->ReadPositions(angles, MotorControl::Config::HEAD);
	HardGet(angles, MotorControl::Config::HEAD);

	angles[0] = angles[0] * M_PI/180;
	angles[1] = std::abs(angles[1] * M_PI/180);

	double euclidianDistance = _robotHeight * std::tan((M_PI/2)-angles[1]);
	ObjectPosition objectPosition;
	objectDistance.x = euclidianDistance * std::sin(angles[0]);
	objectDistance.y = euclidianDistance * std::cos(angles[0]);

	Logger::getInstance() << "Euclidian distance: " << euclidianDistance << " cm" << std::endl;


	return objectDistance;
}
