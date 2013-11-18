#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "Control/STM32F4_2.h"
#include "Control/Motor.h"

#include "Utilities/ThreadManager.h"
#include "Utilities/XmlParser.h"

#include <vector>
#include <boost/thread/mutex.hpp>
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

   MotorControl(std::shared_ptr<ThreadManager> threadManager_ptr, const XmlParser &config);
   ~MotorControl();

   void run();

   bool SetTorque(bool value, const Config config);
   bool SetTorque(bool value, const std::string name);
  
   bool InitPositions( const std::vector<double>& vPos,
                      const Config config,
                      const double msTotalTime = 10000.0,
                      const double msDt = 16);
  
   bool SetPosition(double pos, std::string name); 
   const double ReadPosition(std::string name);
   void Update(const std::vector<char>& msg);
   
   bool SetPositions(const std::vector<double>& pos, const Config config);
   bool ReadPositions(std::vector<double>& pos, const Config config);

   // TODO Do not look
   void HardSet(const std::vector<double>& pos, const Config config);
   void HardGet(std::vector<double>& pos, const Config config);
   void HardGetMaxAngles(std::vector<double>& angles, const Config config);
   void HardGetMinAngles(std::vector<double>& angles, const Config config);

private:
   void InitializeMotors(const XmlParser &config);
   void InitializeConfigurations(const XmlParser &config);

   void Read(const std::vector<char>& msg);
 
   struct ReadData
   {
       char Id;
       std::uint16_t pos;
       char Dt;
       std::uint16_t status;
   } _readData;

   std::shared_ptr<STM32F4> _stm32f4;
   std::shared_ptr<ThreadManager> _threadManager;
   boost::mutex _io_mutex;

   std::map<std::string, Motor*> _mapStrMotors;
   std::map<int, Motor*> _mapIdMotors;
   std::map<Config, std::vector<Motor*>> _configurations;
};
#endif  //MOTOR_CONTROL_H
