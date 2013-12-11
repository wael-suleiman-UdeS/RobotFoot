#ifndef PROTOCOL_H
#define PROTOCOL_H
#include "Utilities/miscutils.h"

#include <cstdint>
#include <vector>

namespace Protocol
{
    ////////////////////////// RobotFoot USB Protocol /////////////////////////////////////
    //
    // The protocol between the embedded computer and the control board works 
    // with tags idendification. Each tag is 2 bytes in size and is always followed
    // by a 2 bytes size.
    // 
    // MAIN STRUCTURE ---------------------------------------------------------------------
    // Each packet(stream) sent and received must be encapsulated
    // by a header tag(0xffff). Each packet sent/received must
    // follow this structure :
    //
    //  0      1 2       3 4      n+1 n+2    n+3
    //  -------- --------- ---------- ----------
    // | 0xffff | Size(n) |   Data   | Checksum |
    //  -------- --------- ---------- ----------
    //          |____________________|
    //                    n
    // - The size is in byte and includes imself and the data section.
    // - The "data" section hold the sub-tags for specific commands.
    // - Only the header tag is ending with a checksum.
    // 
    // ------------------------------------------------------------------------------------
    // The following are sub-tags of the main struture's "Data" section describing
    // specific type of commands.
    //
    //  ==> MOTOR TAG (read/write)---------------------------------------------------------
    //      Various data specific to motors. Tag is "MO" in hex value. 
    //      
    //      0      1 2       3     4      5        6   7   8      9 10 11   12        13 
    //      -------- --------- ---------- ---------- ----- -------- ----- ------ -------------
    //     | 0x4f4d | Size(n) | Motor ID | Position | DT  | Status | PWM | Volt | Temperature |
    //      -------- --------- ---------- ---------- ----- -------- ----- ------ -------------
    //              |_________________________________________________________________________|
    //                                               n
    //       
    //      -------------------------------------------------------------------------------
    //      
    //  ==> GYROSCOPE/ACCELEROMETER TAG (read only)----------------------------------------
    //      Updated Gyro/Acc values. Tag is "GA" in hex value.
    //     
    //      0      1 2       3 4      5 6      7 8     9 10   11
    //      -------- --------- -------- -------- ------- -------
    //     | 0x4147 | Size(n) | Gyro x | Gyro y | Acc x | Acc y |
    //      -------- --------- -------- -------- ------- -------
    //              |___________________________________________|
    //                                 n
    //      -------------------------------------------------------------------------------
    //      
    //  ==> BUTTON TAG (read only)---------------------------------------------------------
    //      Updated button status. Sent only when status change. Tag is "BU" in hex value.
    //
    //      0      1 2       3      4         5
    //      -------- --------- ----------- -------
    //     | 0x5542 | Size(n) | Button ID | Value |
    //      -------- --------- ----------- -------
    //              |_____________________________|
    //                            n
    //      -------------------------------------------------------------------------------
    //
    //  ==> POWER TAG (read only)----------------------------------------------------------
    //      Updated battery level in volt. Value must be divided by 16.0
    //      to give the volt representation. Tag is "PO" in hex value.
    //
    //      0      1 2       3    4     
    //      -------- --------- -------
    //     | 0x4f50 | Size(n) | Value |
    //      -------- --------- -------
    //              |_________________|
    //                        n
    //      -------------------------------------------------------------------------------
    //   
    //  ==> TORQUE TAG (write only)--------------------------------------------------------
    //      Specific command to set (on/off) the motors torque. Tag is "TO" in hex value.
    //
    //      0      1 2       3      4        5
    //      -------- --------- ---------- -------
    //     | 0x4f54 | Size(n) | Motor ID | Value |
    //      -------- --------- ---------- -------
    //              |____________________________|
    //                            n
    //      -------------------------------------------------------------------------------
    //
    //  ==> MOTOR RAW TAG (read/write)-----------------------------------------------------
    //      Direct command to the motors. The data of this command is sent directly to the
    //      motors without validation. Answers to those commands are stacked
    //      in a circular buffer.
    // 
    //      0      1 2       3      4     5   n+1
    //      -------- --------- ---------- -------
    //     | 0x534d | Size(n) | Motor ID | Data  |
    //      -------- --------- ---------- -------
    //              |____________________________|
    //                            n
    //      -------------------------------------------------------------------------------
    ///////////////////////////////////////////////////////////////////////////////////////

    // Protocol Tags
    // TODO Hardcode 
    const uint16le MsgHeader = 0xffff; // Main header tag. 

    const uint16le MotorHeader    = 0x4f4d; // "MO" => Motor status and pos command
    const uint16le GyroAccHeader  = 0x4147; // "GA" => Gyro/Acc Values
    const uint16le ButtonHeader   = 0x5542; // "BU" => Button Status
    const uint16le PowerHeader    = 0x4F50; // "PO" => Battery Power Status
    const uint16le TorqueHeader   = 0x4F54; // "TO" => Motor Torque
    const uint16le MotorRawHeader = 0x534D; // "MR" => Motor Raw commands. Those commands are transmitted directly to the motors.
    
    void Separate2Bytes(const std::uint16_t value, std::uint8_t& valueLSB, std::uint8_t& valueMSB);
    void Unify2Bytes(std::uint16_t& value, const std::uint8_t valueLSB, const std::uint8_t valueMSB);
    bool FindMsgHeader(std::vector<char>::const_iterator &start, const std::vector<char> &msg, uint16le& header);
    
    char CalculCheckSum(const std::vector<char>& msg);
    std::vector<char> GenerateDataMsg(uint16le header, const std::vector<char>& data);
    bool isTag(uint16le header);

    struct MotorStruct
    {
        std::uint8_t    id;
        uint16le       pos;
        std::uint8_t    dt;
        uint16le    status;
        uint16le       PWM;
        std::uint8_t  volt;
        std::uint8_t  temp;
    };

    struct GyroAccStruct
    {
        uint16le gyro_x;
        uint16le gyro_y;
        uint16le acc_x;
        uint16le acc_y;
    };

    struct ButtonStruct
    {
        std::uint8_t    id;
        std::uint8_t value;
    };

    struct PowerStruct
    {
        std::uint8_t value;
    };

    struct TorqueStruct
    {
        std::uint8_t    id;
        std::uint8_t value;
    };
}
#endif //PROTOCOL_H
