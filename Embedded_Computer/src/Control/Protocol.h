#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cstdint>
#include <vector>

namespace Protocol
{
    ////////////////////////// STM32F4 Protocol /////////////////////////////////////
    //
    // Each type of command is identified by a header tag.
    // The main tag representing the start of a packet stream is 0xffff.
    // Each tag are 2 bytes and are followed by a 2 bytes size. 
    //
    // Here is the main packet stream representation :
    //
    //  0      1 2       3 4        n n+1    n+2
    //  -------- --------- ---------- ----------
    // | 0xffff | Size(n) |   Data   | Checksum |
    //  -------- --------- ---------- ----------
    //          |____________________|
    //                     n
    // Only the main packet stream has a checksum.
    // 
    // The data segment is x command(s) each identified by tags.
    // 
    // TODO Completing Doc
    //
    /////////////////////////////////////////////////////////////////////////////////

    // Protocol Tags
    // TODO Hardcode
    const std::uint16_t MsgHeader = 0xffff; // Main header tag. 

    const std::uint16_t MotorHeader    = 0x4f4d; // "MO" => Motor status and pos command
    const std::uint16_t GyroAccHeader  = 0x4147; // "GA" => Gyro/Acc Values
    const std::uint16_t ButtonHeader   = 0x5542; // "BU" => Button Status
    const std::uint16_t PowerHeader    = 0x4F50; // "PO" => Battery Power Status
    const std::uint16_t TorqueHeader   = 0x4F54; // "TO" => Motor Torque
    const std::uint16_t MotorRawHeader = 0x534D; // "MR" => Motor Raw commands. Those commands are transmitted directly to the motors.
    
    void Separate2Bytes(const std::uint16_t value, std::uint8_t& valueLSB, std::uint8_t& valueMSB);
    void Unify2Bytes(std::uint16_t& value, const std::uint8_t valueLSB, const std::uint8_t valueMSB);
    std::uint16_t FindMsgHeader(std::vector<char>::const_iterator &start, const std::vector<char> &msg);
    
    char CalculCheckSum(const std::vector<char>& msg);
    std::vector<char> GenerateDataMsg(std::uint16_t header, const std::vector<char>& data);
    bool isTag(std::uint16_t header);

    void ReadPacket(std::uint16_t header,
                    std::vector<char>::const_iterator& mainItr, 
                    const std::vector<char>::const_iterator& mainEnd,
                    std::vector<char>& result);

    struct MotorStruct
    {
        std::uint8_t  id;
        std::uint16_t pos;
        std::uint8_t  dt;
        std::uint16_t status;
        std::uint16_t PWM;
        std::uint8_t  volt;
        std::uint8_t  temp;
    };

    struct GyroAccStruct
    {
        // TODO
    };

    struct ButtonStruct
    {
        std::uint8_t id;
        std::uint8_t value;
    };

    struct PowerStruct
    {
        std::uint8_t value;
    };

    struct TorqueStruct
    {
        std::uint8_t id;
        std::uint8_t value;
    };
}
#endif //PROTOCOL_H
