#include "STM32F4.h"
#include <boost/ref.hpp>

using std::string;
using boost::asio::io_service;

using std::uint8_t;
using std::uint16_t;

STM32F4::STM32F4(string portName, io_service& io)
	: _usb(boost::ref(io),
	  portName.c_str(), 115200)
{
}

STM32F4::~STM32F4()
{
}

void STM32F4::setMotor(uint8_t id, uint16_t value, uint8_t playTime/* = 70*/)
{
	std::vector<char> msg;
	const uint8_t cmd = 0x01;
	const uint8_t highPos = value >> 8;
	const uint8_t lowPos = value;
	const uint8_t nb = 5;
	const uint8_t checkSum = cmd + id + highPos + lowPos + playTime + nb;

	msg.push_back('\xff');
	msg.push_back(checkSum);
	msg.push_back(nb);
	msg.push_back(cmd);
	msg.push_back(id);
	msg.push_back(highPos);
	msg.push_back(lowPos);
    msg.push_back(playTime);
	_usb.write(msg);
}

void STM32F4::setTorque(uint8_t id, TorqueState state)
{
	std::vector<char> msg;
	const uint8_t cmd = 0x03;
	const uint8_t nb = 3;
	const uint8_t checkSum = cmd + nb + uint8_t(state) + id;

	msg.push_back('\xff');
	msg.push_back(checkSum);
	msg.push_back(nb);
	msg.push_back(cmd);
	msg.push_back(id);
	msg.push_back(uint8_t(state));
   _usb.write(msg);
}

int STM32F4::read(uint8_t id)
{
	std::vector<char> msg;
	const uint8_t cmd = 0x02;
	const uint8_t nb = 2;
	const uint8_t checkSum = cmd + nb + id;

	msg.push_back('\xff');
	msg.push_back(checkSum);
	msg.push_back(nb);
	msg.push_back(cmd);
	msg.push_back(id);

	msg = _usb.read_sync(msg);

	if (msg.size() < 4) { return 0; }

	return ((msg[2] & 0xFF) << 8) | (msg[3] & 0xFF);  
}

int STM32F4::readStatus(std::uint8_t id)
{
	std::vector<char> msg;
	const uint8_t cmd = 0x04;
	const uint8_t nb = 2;
	const uint8_t checkSum = cmd + nb + id;

	msg.push_back('\xff');
	msg.push_back(checkSum);
	msg.push_back(nb);
	msg.push_back(cmd);
	msg.push_back(id);
	
	msg = _usb.read_sync(msg);

	if (msg.size() < 3) { return 0; }

	return msg[2];
}

void STM32F4::clearStatus(std::uint8_t id)
{
	std::vector<char> msg;
	const uint8_t cmd = 0x05;
	const uint8_t nb = 2;
	const uint8_t checkSum = cmd + nb + id;

	msg.push_back('\xff');
	msg.push_back(checkSum);
	msg.push_back(nb);
	msg.push_back(cmd);
	msg.push_back(id);
   _usb.write(msg);
}

void STM32F4::writeRAM(std::uint8_t id, std::uint8_t adress, std::uint8_t value)
{
	std::vector<char> msg;
	const uint8_t cmd = 0x06;
	const uint8_t nb = 4;
	const uint8_t checkSum = cmd + nb + id + adress + value;

	msg.push_back('\xff');
	msg.push_back(checkSum);
	msg.push_back(nb);
	msg.push_back(cmd);
	msg.push_back(id);
    msg.push_back(adress);
    msg.push_back(value);
   _usb.write(msg);
}

int STM32F4::readRAM(std::uint8_t id, std::uint8_t adress)
{
	std::vector<char> msg;
	const uint8_t cmd = 0x07;
	const uint8_t nb = 3;
	const uint8_t checkSum = cmd + nb + id + adress;

	msg.push_back('\xff');
	msg.push_back(checkSum);
	msg.push_back(nb);
	msg.push_back(cmd);
	msg.push_back(id);
    msg.push_back(adress);
	
	msg = _usb.read_sync(msg);

	if (msg.size() < 3) { return 0; }

	return msg[2];
}

