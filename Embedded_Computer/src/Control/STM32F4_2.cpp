#include "Control/STM32F4_2.h"

#include "Control/Protocol.h"

#include <boost/ref.hpp>

using boost::asio::io_service;

    STM32F4::STM32F4(std::string portName, io_service& io)
: _usb(boost::ref(io),
        portName.c_str(), 115200)
{
}

STM32F4::~STM32F4()
{
}

void STM32F4::RegisterRead(/*lambda*/)
{
    // TODO
}

void STM32F4::AddMsg(const std::vector<char>& msg)
{
    _data.insert(_data.end(), msg.begin(), msg.end());
}

void STM32F4::SendMsg()
{
    std::vector<char> msg;
   
    Protocol::GenerateDataMsg(Protocol::MsgHeader, _data, msg);

    const uint8_t checkSum = Protocol::CalculCheckSum(msg);
    msg.push_back(checkSum);
   
    _usb.write(msg);
    _data.clear();
}

