#include "Control/Protocol.h"
#include "Control/Motor.h"
#include "Utilities/logger.h"
#include "Utilities/miscutils.h"

#include <boost/algorithm/clamp.hpp>
#include <vector>

using boost::algorithm::clamp;

namespace
{
	const double dAngleConvertion = 0.325;
	const double dInvAngleConvertion = 1/dAngleConvertion;
}

Motor::Motor(std::shared_ptr<STM32F4> stm32f4, std::string name, int id, int offset, int minValue, int maxValue, int playTime)
:
_stm32f4(stm32f4),
_name(name),
_id(id),
_offset(offset),
_minValue(minValue),
_maxValue(maxValue),
_playTime(playTime),
_angle(0.0),
_torque(0)
{    
}

Motor::~Motor()
{
}

void Motor::setPos(double pos)
{
    if(_angle != pos && _torque)
    {
        std::vector<char> data;
        data.push_back(_id);

        uint16le posValue = Angle2Value(pos);
        data.push_back(posValue.bytes[0]);
        data.push_back(posValue.bytes[1]);
        data.push_back(_playTime); 

        _stm32f4->AddMsg(Protocol::GenerateDataMsg(Protocol::MotorHeader,data));
    }    
}

void Motor::sendRawPacket(const std::uint8_t& cmd, const std::vector<char>& data)
{
    std::vector<char> msg;
    msg.push_back(_id);
    msg.push_back(cmd);
    msg.insert(msg.end(), data.begin(), data.end());

    _stm32f4->AddMsg(Protocol::GenerateDataMsg(Protocol::MotorRawHeader, msg));
}

void Motor::setTorque(bool value)
{
    if (_torque != value)
    {
        _torque = value;
        std::vector<char> data;
        data.push_back(_id);
        data.push_back(value);

        _stm32f4->AddMsg(Protocol::GenerateDataMsg(Protocol::TorqueHeader, data));
    }
}

void Motor::update(const Protocol::MotorStruct &motorStruct)
{
    _angle = Value2Angle(motorStruct.pos);
    _status = motorStruct.status;
    _PWM = motorStruct.PWM;
    _volt = motorStruct.volt;
    _temp = motorStruct.temp;
}

const Protocol::MotorStruct Motor::getStatus()
{
    Protocol::MotorStruct motorInfo;
    motorInfo.id = _id;
    motorInfo.pos = Angle2Value(_angle);
    motorInfo.status = _status;
    motorInfo.PWM = _PWM;
    motorInfo.volt = _volt;
    motorInfo.temp = _temp;
    return motorInfo;
}

const double Motor::getPos()
{
    return _angle;
}

std::uint16_t Motor::Angle2Value(const double angle)
{
    std::uint16_t value = (angle*dInvAngleConvertion) + _offset;
	return clamp(value, _minValue, _maxValue);
}

double Motor::Value2Angle(const std::uint16_t value)
{
    std::uint16_t clampedValue = clamp(value, _minValue, _maxValue);
	return double(clampedValue - _offset)*dAngleConvertion;
}

const double Motor::getMinAngle()
{
	return Value2Angle(_minValue);
}

const double Motor::getMaxAngle()
{
	return Value2Angle(_maxValue);
}
