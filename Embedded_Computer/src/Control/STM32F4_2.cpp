#include "Control/STM32F4_2.h"
#include "Control/Protocol.h"

#include <boost/ref.hpp>

using boost::asio::io_service;

STM32F4::STM32F4(std::string portName, io_service& io, std::function<void(std::vector<char>)> &function)
: 
    _usb(boost::ref(io), portName.c_str(), 115200),
    _callBackFunction(std::make_share(function))
{
    _usb.start_read(boost::bind(&STM32F4::ReceiveMsg, this));
}

STM32F4::~STM32F4()
{
}

void STM32F4::AddMsg(const std::vector<char>& msg)
{
    _outBuffer.insert(_outBuffer.end(), msg.begin(), msg.end());
}

void STM32F4::SendMsg()
{
    std::vector<char> msg;
   
    Protocol::GenerateDataMsg(Protocol::MsgHeader, _outBuffer, msg);

    const uint8_t checkSum = Protocol::CalculCheckSum(msg);
    msg.push_back(checkSum);
   
    _usb.write(msg);
    _outBuffer.clear();
}

void ReceiveMsg(const std::vector<char> msg)
{
    auto msg_header = Protocol::findHeader(msg);
    if () // no size
    {
           
    }
    if ()
    {
        _callBackFunction(_inBuffer);
    }
}
