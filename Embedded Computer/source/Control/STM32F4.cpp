#include "STM32F4.h"

using std::string;
using boost::asio::io_service;

using std::uint8_t;

// portname = argc > 1 ? std::string("/dev/") + argv[1] : std::string("/dev/ttyUSB0")
STM32F4::STM32F4(string portName, io_service& io)
	: _usb(boost::ref(io),
	  portName.c_str(), 115200)
{
}

STM32F4::~STM32F4()
{
}

void STM32F4::setMotor(uint8_t id, uint8_t value)
{
	std::stringstream ss;
	ss << 0x01 << id << value;
	_usb.write(ss.str().c_str());
}