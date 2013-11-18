#include "Control/Protocol.h"
#include "Utilities/logger.h"

#include <boost/algorithm/clamp.hpp>
#include <vector>

using boost::algorithm::clamp;

namespace
{
	const double dAngleConvertion = 0.325;
	const double dInvAngleConvertion = 1/dAngleConvertion;
}

Motor::Motor(std::shared_ptr<STM32F4> stm32f4, std::string name, int id, int offset, int min, int max, int playTime)
:
_stm32f4(stm32f4),
_name(name),
_id(id),
_offset(offset),
_min(min),
_max(max),
_playTime(playTime),
_angle(0.0)
{    
}

Motor::~Motor()
{
}

void Motor::setPos(double pos)
{
    if(_angle != pos)
    {
        _angle = pos;
        std::vector<char> data;
        data.push_back(_id);

        char posLSB, posMSB;
        int pos = Angle2Value(_posToWrite);
        Protocol::Separate2Bytes(pos, posLSB, posMSB);
        data.push_back(posLSB);
        data.push_back(posMSB);
        data.push_back(_playTime); 

        std::vector<char> msg;
        Protocol::GenerateDataMsg(Protocol::MotorHeader,data,msg);
        _stm32f4->AddMsg(msg);
    }    
}

const double Motor::getPos()
{
    return _posToRead;
}

int Motor::Angle2Value(const double angle)
{
    int value = (angle*dInvAngleConvertion) + _offset;
	return clamp(value, _min, _max);
}

double Motor::Value2Angle(const int value)
{
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
