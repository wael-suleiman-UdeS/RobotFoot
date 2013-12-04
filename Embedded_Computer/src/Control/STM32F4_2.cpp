#include "Control/STM32F4_2.h"
#include "Control/Protocol.h"
#include "Utilities/logger.h"
#include "Utilities/miscutils.h"

using boost::asio::io_service;

STM32F4::STM32F4(std::string portName, io_service& io, std::function<void(std::vector<char>)> function)
: 
    _usb(io, portName.c_str(), 115200),
    _callBackFunction(function)
{
    _usb.start_read([this](std::vector<char> p) { ReceiveMsg(p); });
}

STM32F4::~STM32F4()
{
}

void STM32F4::AddMsg(const std::vector<char>& msg)
{
	boost::mutex::scoped_lock lock(_mutex);
    _outBuffer.insert(_outBuffer.end(), msg.begin(), msg.end());
}

void STM32F4::SendMsg()
{
	boost::mutex::scoped_lock lock(_mutex);
    std::vector<char> msg = Protocol::GenerateDataMsg(Protocol::MsgHeader, _outBuffer);

    const uint8_t checkSum = Protocol::CalculCheckSum(msg);
    msg.push_back(checkSum);
   
    _usb.write_async(msg);
    _outBuffer.clear();
}

// TODO Move to Protocol class???
void STM32F4::ReceiveMsg(const std::vector<char> stream)
{
    boost::mutex::scoped_lock lock(_mutex);

    // Add new data to input buffer
    _inBuffer.insert(_inBuffer.end(), stream.begin(), stream.end());

    auto header_it = _inBuffer.cbegin();
    uint16le header = 0;
    while (header_it < _inBuffer.cend() && Protocol::FindMsgHeader(header_it, _inBuffer, header))
    {
        if (header == 0xffff)
        {
            //Logger::getInstance(Logger::LogLvl::DEBUG) << "Parsing 0xFFFF" << std::endl;
            // Dump everything before header and validate msg
            if (header_it + 4 < _inBuffer.cend())
            {
                uint16le size = 0;
                auto size_it = header_it + 2;
                size.bytes[0] = *(size_it);
                size.bytes[1] = *(size_it+1);

                if ((size_it + size) < _inBuffer.cend())
                {
                    std::vector<char> msg(header_it, size_it + size);

                    // Validate checksum
                    if (Protocol::CalculCheckSum(msg) == *(size_it + size))
                    {
                        header_it = header_it + msg.size();
                        _callBackFunction(msg);
                    }
                    else
                    {
                        Logger::getInstance(Logger::LogLvl::DEBUG) << "STM32F4: Checksum validation failed." << std::endl;
                    }
                }
                else
                    break;
            }
            else
                break;
        }
        header_it++;
    }
   
    if (header_it < _inBuffer.cend())
    {
        _inBuffer = std::vector<char>(header_it, _inBuffer.cend());
    }
    else
    {
        _inBuffer.clear();
    }
}
