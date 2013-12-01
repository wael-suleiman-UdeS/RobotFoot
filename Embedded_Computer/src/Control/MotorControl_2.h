#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "Control/STM32F4_2.h"
#include "Control/Motor.h"
#include "Control/Protocol.h"

#include "Utilities/ThreadManager.h"
#include "Utilities/XmlParser.h"

#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <memory> // shared_ptr

class MotorControl
{
public:
   enum class Config {
      ALL_MOTORS = 0,
      ALL_LEGS,
      RIGHT_LEG,
      LEFT_LEG,
      HEAD,
      NUM_TEST
   };

   enum class Button {
      BUTTON_1 = 0,
      BUTTON_2,
      BUTTON_3,
      BUTTON_4
   };

   MotorControl(std::shared_ptr<ThreadManager> threadManager_ptr, const XmlParser &config, boost::asio::io_service &boost_io);
   ~MotorControl();

   void run(int ms_sleepTime);
   void WriteAll();
   bool InitPositions( const std::vector<double>& vPos,
                      const Config config,
                      const double msTotalTime = 10000.0,
                      const double msDt = 16);
  
   void UpdateMotorStatus(const std::vector<char>& msg);
   void GetMotorStatus(std::vector<Protocol::MotorStruct> &status, const Config config);
   bool GetButtonStatus(const Button button_enum);

   bool SetTorque(bool value, const Config config);

   bool SetPosition(double pos, std::string name); 
   bool SetPositions(const std::vector<double>& pos, const Config config);
   
   const double ReadPosition(std::string name);
   bool ReadPositions(std::vector<double>& pos, const Config config);

   void SendRawPacket(const std::uint8_t& cmd, const std::vector<char>& data, std::string name);
   void SendRawPackets(const std::vector<std::uint8_t>& cmds, const std::vector<std::vector<char>>& data, const Config config);
   std::vector<char> GetRawPacket();

   // TODO To be removed
   void HardSet(const std::vector<double>& pos, const Config config);
   void HardGet(std::vector<double>& pos, const Config config);
   void HardGetMaxAngles(std::vector<double>& angles, const Config config);
   void HardGetMinAngles(std::vector<double>& angles, const Config config);

private:
   void InitializeMotors(const XmlParser &config);
   void InitializeConfigurations(const XmlParser &config);

   std::vector<bool> _buttonStatus;

   std::shared_ptr<STM32F4> _stm32f4;
   std::shared_ptr<ThreadManager> _threadManager;
   
   boost::mutex _io_mutex;
   boost::circular_buffer<std::vector<char>> _rawPackets;

   std::map<std::string, Motor*> _mapStrMotors;
   std::map<int, Motor*> _mapIdMotors;
   std::map<Config, std::vector<Motor*>> _configurations;
};
#endif  //MOTOR_CONTROL_H
