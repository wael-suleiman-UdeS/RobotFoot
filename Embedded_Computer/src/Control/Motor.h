#ifndef MOTOR_H
#define MOTOR_H

#include <memory> // shared_ptr
#include "Control/Protocol.h"
#include "Control/STM32F4_2.h"

class Motor
{
    public:  
        Motor(std::shared_ptr<STM32F4> stm32f4, std::string name, int id, int offset, int minValue, int maxValue, int playTime);
        ~Motor();
        void setPos(double pos);
        void setTorque(bool value);
        void update(const Protocol::MotorStruct &motorStruct);
        void sendRawPacket(const std::uint8_t& cmd, const std::vector<char>& data);

        const double getPos();
        const double getMinAngle();
        const double getMaxAngle();
        const Protocol::MotorStruct getStatus();

    private:
        std::uint16_t Angle2Value(const double angle);
        double Value2Angle(const std::uint16_t value);

        std::shared_ptr<STM32F4> _stm32f4; 
        std::string _name;
        int _id;
        int _offset;
        std::uint16_t _minValue;
        std::uint16_t _maxValue;
        int _speed;
        int _playTime;
        double _angle;
        bool _torque;
        std::uint16_t _status;
        std::uint16_t _PWM;
        std::uint16_t _volt;
        std::uint16_t _temp;
};
#endif  //MOTOR_H

