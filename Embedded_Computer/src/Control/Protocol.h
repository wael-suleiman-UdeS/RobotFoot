#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cstdint>
#include <vector>

namespace Protocol
{
    const std::int16_t MsgHeader = 0xffff;
    const std::int16_t MotorHeader = 0x4d4f; //"MO"
    const std::int16_t GiroAccHeader = 0x4741; //"GA"
    const std::int16_t ButtonHeader = 0x4255; //"BU"
    const std::int16_t PowerHeader = 0x504f; //"PO"
    
    void Separate2Bytes(const std::int16_t value, char& valueLSB, char& valueMSB);
    void Unify2Bytes(std::int16_t& value, const char valueLSB, const char valueMSB);
    
    char CalculCheckSum(const std::vector<char>& msg);

    void GenerateDataMsg(std::int16_t header, const std::vector<char>& data, std::vector<char>& result);

//    int ReadMsg(std::int16_t header, const std::vector<char>& msg, std::vector<char>& result);
    void ReadPacket(std::int16_t header,
            std::vector<char>::const_iterator& mainItr, 
            const std::vector<char>::const_iterator& mainEnd,
            std::vector<char>& result);

}

#endif //PROTOCOL_H
