#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "Utilities/ThreadManager.h"
#include "Control/STM32F4.h"
#include "Utilities/XmlParser.h"

#include <vector>
#include <boost/thread/mutex.hpp>
#include <memory> // shared_ptr

class Motor
{
    public:  
        Motor(STM32F4 *stm32f4, std::string name, int id, int offset, int min, int max, int playTime, bool isInversed);
        ~Motor();
        void setPos(double pos);
        const double getPos();
        const double getMinAngle();
        const double getMaxAngle();
        void setTorque(bool value);
        void Read();
        void Write();
    private:
        int Angle2Value(const double angle);
        double Value2Angle(const int value);
        
        STM32F4 *_stm32f4; 
        std::string _name;
        int _id;
        int _offset;
        int _min;
        int _max;
        int _speed;
        int _playTime;
        double _lastPos;
        double _currentPos;
        bool _isInversed;
};

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

   void ReadAll();
public:
   void WriteAll();
private:

   STM32F4 *_stm32f4;
   std::shared_ptr<ThreadManager> _threadManager;
   boost::mutex _io_mutex;

   std::map<std::string, Motor*> _motors;
   std::map<Config, std::vector<Motor*>> _configurations;
};
#endif  //MOTOR_CONTROL_H
