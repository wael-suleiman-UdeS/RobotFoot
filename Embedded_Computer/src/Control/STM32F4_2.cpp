#include "STM32F4_2.h"
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

}

void STM32F4::AddMsg(const std::vector<std::uint8_t>& msg)
{
   _msg.insert(_msg.end(), msg.begin(), msg.end());
}

void STM32F4::SendMsg()
{
   _usb.write(_msg);
   _msg.clear();
}

