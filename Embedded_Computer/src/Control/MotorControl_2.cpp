#include "Control/MotorControl_2.h"
#include "Control/Protocol.h"
#include "Control/Motor.h"
#include "Utilities/logger.h"

#include <iostream>
#include <iterator>
#include <algorithm>
#include <functional>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#define DANGER_TEST_MOTION
//#define DEBUG_TEST_MOTION

using boost::filesystem::path;

MotorControl::MotorControl(std::shared_ptr<ThreadManager> threadManager_ptr, const XmlParser &config, boost::asio::io_service &boost_io) :
 _buttonStatus(3, false),
 _threadManager(threadManager_ptr),
 _rawPackets(20),
 _isPaused(true),
 _currentColor("red")
{
    try
    {
        // Init USB interface with STM32F4
        Logger::getInstance() << "Initializing USB interface..." << std::endl;
        std::string port_name = config.getStringValue(XmlPath::Root / "USB_Interface" / "TTY");
        _robotHeight = config.getIntValue(XmlPath::Root / XmlPath::Sizes / "RobotHeight");
        _stm32f4 = std::make_shared<STM32F4>(port_name, boost_io, [this](std::vector<char> a) { return UpdateMotorStatus(a); });

        _trackingObjects = config.getChildrenStringValues(XmlPath::Root / XmlPath::ImageProcessing / XmlPath::Objects);
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
        std::shared_ptr<Motor> motor(new Motor(_stm32f4, it->first, id, offset, min, max, playTime, isInversed));
	    _mapStrMotors.insert(std::make_pair(it->first, motor));
        _mapIdMotors.insert(std::make_pair(id,motor));
    }
    
    InitPID(config);
}

void MotorControl::InitPID(const XmlParser &config)
{
    /*
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
    */
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
        std::vector<std::shared_ptr<Motor>> motors;
        std::vector<std::string> names = config.getChildrenStringValues(it->second);
        for (auto name = names.begin(); name != names.end(); ++name)
        {
            if (_mapStrMotors.find(*name) != _mapStrMotors.end())
                motors.push_back(_mapStrMotors[*name]);
        } 
        _configurations.insert(std::make_pair(it->first, motors));
    }
}

// Update motors status 
void MotorControl::UpdateMotorStatus(const std::vector<char>& msg)
{
    auto header_it = msg.cbegin() + 1; // +1 to skip 0xffff tag
    uint16le header = 0;
    while (Protocol::FindMsgHeader(header_it, msg, header))
    {
        if (header_it < msg.cend() - 4)
        {
            auto size_it = header_it + 2;
            uint16le msgSize = 0;
            msgSize.bytes[0] = *size_it;
            msgSize.bytes[1] = *(size_it+1);
           
            auto data_start = header_it + 4;
            auto data_end   = size_it + msgSize;
            if (header == Protocol::MotorHeader)
            {
                //Logger::getInstance(Logger::LogLvl::DEBUG) << "Parsing MO" << std::endl;
                Protocol::MotorStruct motorStruct;
                size_t size = std::min(msgSize * sizeof(char), sizeof(Protocol::MotorStruct));
                memcpy(&motorStruct, &*data_start, size);

                auto motor = _mapIdMotors.find(motorStruct.id);
                if (motor != _mapIdMotors.end())
                {
                    motor->second->update(motorStruct);
                }
            }
            else if (header == Protocol::GyroAccHeader)
            {
               // TODO
               // Protocol::GyroAccStruct gyroAccStruct;
               // size_t size = std::min(msgSize * sizeof(char), sizeof(Protocol::GyroAccStruct));
               // memcpy(&gyroAccStruct, &*data_it, size);
            }
            else if (header == Protocol::ButtonHeader)
            {
                Protocol::ButtonStruct buttonStruct;
                size_t size = std::min(msgSize * sizeof(char), sizeof(Protocol::ButtonStruct));
                memcpy(&buttonStruct, &*data_start, size);

                if (buttonStruct.id >= 0 && buttonStruct.id < _buttonStatus.size())
                {
                    if (buttonStruct.id == 2 && buttonStruct.value >= 1)
                    {
                        _isPaused = !_isPaused;
                        if (_isPaused)
                        {
                            _threadManager->stop(ThreadManager::Task::HEAD_CONTROL);
                            _threadManager->stop(ThreadManager::Task::LEGS_CONTROL);
                        }
                    }
                    _buttonStatus[buttonStruct.id] = buttonStruct.value;
                    Logger::getInstance(Logger::LogLvl::INFO) << "Button " <<(int) buttonStruct.id << " value = " <<(int) buttonStruct.value << std::endl;
                }
            }
            else if (header == Protocol::PowerHeader)
            {
                // TODO
                //Protocol::PowerStruct powerStruct;
                //size_t size = std::min(msgSize * sizeof(char), sizeof(Protocol::PowerStruct));
                //memcpy(&powerStruct, &*data_it, size);
            }
            else if (header == Protocol::MotorRawHeader)
            {
                if (msgSize > 4)
                {
                    _rawPackets.push_back(std::vector<char>(data_start, data_end)); 
                }                
            }
        }
        header_it++;
    }
}

void MotorControl::GetMotorStatus(std::vector<Protocol::MotorStruct> &status, const Config config)
{ 
   if (_configurations.find(config) == _configurations.end())
   {
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"GetStatus\" : Configuration is invalid." << std::endl;
   }

   auto it = _configurations[config].begin();
   const auto end = _configurations[config].end();
   for ( ; it != end; it++ )
   {
       status.push_back((*it)->getStatus());
   }
}

bool MotorControl::GetButtonStatus(const Button button_enum)
{
   return _buttonStatus[(int)button_enum];
}
   
bool MotorControl::isPaused()
{
   return _isPaused; 
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

   boost::this_thread::sleep(boost::posix_time::milliseconds(20)); // wait for motors to update
   std::vector<double> pos;
   if (!ReadPositions(pos, config))
   {
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"InitPosition\" : Error ready positions.";
       return false;
   }
#ifdef DEBUG_TEST_MOTION
   Logger::getInstance(Logger::LogLvl::DEBUG) << "";
   std::copy(pos.begin(), pos.end(), std::ostream_iterator<double>(std::cout, " "));
   Logger::getInstance(Logger::LogLvl::DEBUG) << std::endl;
#endif   
   
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
    if (_mapStrMotors.find(name) == _mapStrMotors.end())
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"SetPosition\" : Motor " << name << " invalid." << std::endl;
        return false;
    }

    _mapStrMotors[name]->setPos(pos);
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

   bool status = true; //TODO Do something with status of motors

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

bool MotorControl::SetTorque(bool value, const Config config)
{
   if (_configurations.find(config) == _configurations.end())
   {
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"SetTorque\" : Configuration is invalid." << std::endl;
       return false;
   }
   auto itrJoint = _configurations[config].begin();
   const auto endJoint = _configurations[config].end();
   
   for ( ; itrJoint != endJoint; itrJoint++)
   {
       (*itrJoint)->setTorque(value);
   }
   return true;
}

const double MotorControl::ReadPosition(std::string name)
{
    if (_mapStrMotors.find(name) == _mapStrMotors.end())
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"ReadPosition\" : Motor " << name << " invalid." << std::endl;
        return -1;
    }
    return _mapStrMotors[name]->getPos();
}

bool MotorControl::ReadPositions(std::vector<double>& pos, const Config config)
{
   if (_configurations.find(config) == _configurations.end())
   {
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"ReadPositions\" : Configuration is invalid." << std::endl;
       return false;
   }

   bool status = true; //TODO Do something with status of motors
   auto itr = _configurations[config].begin();
   const auto end = _configurations[config].end();

   for ( ; itr != end && status ; itr++ )
   {
       pos.push_back((*itr)->getPos());
   }
   return status;
}

void MotorControl::SendRawPacket(const std::uint8_t& cmd, const std::vector<char>& data, std::string name)
{
    if (_mapStrMotors.find(name) == _mapStrMotors.end())
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"SendRawPacket\" : Motor " << name << " invalid." << std::endl;
        return;
    }
    _mapStrMotors[name]->sendRawPacket(cmd, data);
}

void MotorControl::SendRawPackets(const std::vector<std::uint8_t>& cmds, const std::vector<std::vector<char>>& data, const Config config) 
{
   if (_configurations.find(config) == _configurations.end())
   {
       Logger::getInstance(Logger::LogLvl::ERROR) << "In function \"SendRawPackets\" : Configuration is invalid." << std::endl;
       return;
   }

   auto config_it = _configurations[config].begin();
   auto cmd_it = cmds.cbegin();
   auto data_it = data.cbegin();
   const auto end = _configurations[config].end();

   for ( ; config_it != end; ++config_it, ++cmd_it, ++data_it)
   {
       (*config_it)->sendRawPacket(*cmd_it, *data_it);
   }
}

std::vector<char> MotorControl::GetRawPacket()
{
    std::vector<char> packet;
    if (!_rawPackets.empty())
    {
        packet = _rawPackets.back();
        _rawPackets.pop_back();
    }
    return packet;
}

void MotorControl::WriteAll()
{
    _stm32f4->SendMsg();
}

void MotorControl::HardSet(const std::vector<double>& pos, const Config config)
{  
   boost::mutex::scoped_lock lock(_io_mutex);
   auto itrJoint = _configurations[config].begin();
   const auto endJoint = _configurations[config].end();
   auto itrPos = pos.begin();
   const auto endPos = pos.end();

   for ( ; itrJoint != endJoint && itrPos != endPos; itrJoint++, itrPos++ )
   {
       (*itrJoint)->setPos(*itrPos);
   }
   _stm32f4->SendMsg();
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
   for ( ; itrJoint != endJoint; itrJoint++)
   {
       pos.push_back((*itrJoint)->getPos());
   }
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

string MotorControl::GetColorToTrack()
{
   return _currentColor;
}

void MotorControl::ComputeAngle(ObjectPosition object_1, ObjectPosition object_2) 
{
	ObjectPosition distance;
	distance.x = object_2.x - object_1.x;
	distance.y = object_2.y - object_1.y;

	if (object_1.x < 0) {
		// Fuck off
	}
	else if (object_1.y < 0) {
		// 180 - std::atan(std::abs(distance.y / distance.x));
	}
	else {
		// 180 - std::atan(std::abs(distance.x / distance.y));
	}
}

/*
void MotorControl::SetObjectDistance(double xDistance, double yDistance)
{
	//todo
	ObjectPosition objectPosition;
	objectPosition.x = xDistance;
	objectPosition.y = yDistance;
	// if enum = ball
	_ballDistance = objectPosition;
	// else if enum = goal
	_goalDistance = objectPosition;
}

	return objectDistance;
}*/

void MotorControl::SetObjectToTrack(Object object)
{
    if (_trackingObjects.size() > (int) object)
    {
        _objectDistance.x = 0;
        _objectDistance.y = 0;
        _objectDistance.angle = 0;
        _currentColor = _trackingObjects[(int)object];
    }
}
   
void MotorControl::SetObjectPosition(ObjectPosition object)
{
    _objectDistance = object;
}

ObjectPosition MotorControl::GetObjectPosition()
{
    return _objectDistance;
}
