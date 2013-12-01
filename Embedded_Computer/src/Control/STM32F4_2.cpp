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
    _outBuffer.insert(_outBuffer.end(), msg.begin(), msg.end());
}

void STM32F4::SendMsg()
{
    std::vector<char> msg = Protocol::GenerateDataMsg(Protocol::MsgHeader, _outBuffer);

    const uint8_t checkSum = Protocol::CalculCheckSum(msg);
    msg.push_back(checkSum);
   
    _usb.write_async(msg);
    _outBuffer.clear();
}

// TODO Move to Protocol class???
void STM32F4::ReceiveMsg(const std::vector<char> &msg)
{
    boost::mutex::scoped_lock lock(io_mutex);

    // Add new data to input buffer
    _inBuffer.insert(_inBuffer.end(), msg.begin(), msg.end());

    auto header_it = _inBuffer.cbegin();
    uint16le header = 0;
    while (Protocol::FindMsgHeader(header_it, _inBuffer, header) && header == 0xffff)
    {
        // Dump everything before header and validate msg
        _inBuffer = std::vector<char>(header_it, _inBuffer.cend());
        if (_inBuffer.size() > 4)
        {
            uint16le size = 0;
            size.bytes[0] = _inBuffer[2];
            size.bytes[1] = _inBuffer[3];

            if (size <= _inBuffer.size() - 2)
            {
                std::vector<char> msg(_inBuffer.begin(), _inBuffer.begin() + size + 3);

                // Validate checksum
                if (Protocol::CalculCheckSum(msg) == msg.back())
                {
                    // Clear input buffer of the parsed msg
                    _inBuffer = std::vector<char>(_inBuffer.begin() + msg.size(), _inBuffer.end());
                    header_it = _inBuffer.begin() - 1; 
                    _callBackFunction(msg);
                    break; 
                }
                else
                {
                    Logger::getInstance(Logger::LogLvl::DEBUG) << "STM32F4: Checksum validation failed." << std::endl;
                }
            }
        }
        header_it++;
    }
    _inBuffer.clear();
}
