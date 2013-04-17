#include "STM32F4.h"
#include <boost/ref.hpp>

using std::string;
using boost::asio::io_service;

using std::uint16_t;

// portname = argc > 1 ? std::string("/dev/") + argv[1] : std::string("/dev/ttyUSB0")
STM32F4::STM32F4(string portName, io_service& io)
	: _usb(boost::ref(io),
	  portName.c_str(), 115200)
{
}

STM32F4::~STM32F4()
{
}

void STM32F4::setMotor(uint8_t id, uint16_t value)
{
	std::vector<char> msg;

	msg.push_back('\x01');
	msg.push_back(id);
	msg.push_back(value >> 8);
	msg.push_back(value);

	_usb.write(msg);
}

void STM32F4::setTorque(uint8_t id, TorqueState state)
{
	std::vector<char> msg;

	msg.push_back('\x03');
	msg.push_back(id);
	msg.push_back(state);

	_usb.write(msg);
}

void STM32F4::read()
{
	//_usb.read_sync();
}