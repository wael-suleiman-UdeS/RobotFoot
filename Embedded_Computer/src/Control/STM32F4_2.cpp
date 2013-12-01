#include "Control/STM32F4_2.h"
#include "Control/Protocol.h"
#include "Utilities/logger.h"

using boost::asio::io_service;

STM32F4::STM32F4(std::string portName, io_service& io, std::function<void(std::vector<char>)> function)
: 
    _usb(io, portName.c_str(), 115200),
    _callBackFunction(function)
{
   // _usb.start_read(boost::bind(&STM32F4::ReceiveMsg, this));
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

// TODO This is bad coding, should split in smaller and easier to read blocks.
// TODO Move to Protocol class???
void STM32F4::ReceiveMsg(const std::vector<char> &msg)
{
    boost::mutex::scoped_lock lock(io_mutex);

    // Add new data to input buffer
    _inBuffer.insert(_inBuffer.end(), msg.begin(), msg.end());

    auto header_it = _inBuffer.cbegin();
    while (header_it != _inBuffer.cend())
    {
        if (Protocol::FindMsgHeader(header_it, _inBuffer) == 0xffff)
        {
            // Dump everything before header and validate msg
            _inBuffer = std::vector<char>(header_it, _inBuffer.cend());
            if (_inBuffer.size() > 4)
            {
                std::uint16_t size = 0;
                Protocol::Unify2Bytes(size, _inBuffer[2], _inBuffer[3]);
                if ((size_t)size <= _inBuffer.size() - 2)
                {
                    std::vector<char> msg(_inBuffer.begin(), _inBuffer.begin() + size + 3);

                    // Clear input buffer of the parsed msg
                    _inBuffer = std::vector<char>(_inBuffer.begin() + msg.size(), _inBuffer.end()); 
                    
                    // Validate checksum
                    if (Protocol::CalculCheckSum(msg) == msg.back())
                    {
                        _callBackFunction(msg);
                        break; 
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
        else
            header_it++;
    }
    _inBuffer.clear();
}
