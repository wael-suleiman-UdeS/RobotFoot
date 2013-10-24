/**
  ******************************************************************************
  * @file    internals.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-04
  * @brief   Should not be used by user code
  ******************************************************************************
  */
#ifndef INTERNALS_HPP
#define INTERNALS_HPP
//------------------------------------------------------------------------------
#include "miscutils.h"
#include <cstddef>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Header
#define HEADER         0xFF

// Request Packet [To Servo Module]
#define CMD_EEP_WRITE  0x01    // Write Length number of values to EEP Register Address
#define CMD_EEP_READ   0x02    // Request Length number of values from EEP Register Address
#define CMD_RAM_WRITE  0x03    // Write Length number of values to RAM Register Address
#define CMD_RAM_READ   0x04    // Request Lenght number of values from RAM Register Address
#define CMD_I_JOG      0x05    // Able to send JOG command to maximum 43 servos (operate timing of individual Servo)
#define CMD_S_JOG      0x06    // Able to send JOG command to maximum 53 servos (operate simultaneously at same time)
#define CMD_STAT       0x07    // Status Error, Status Detail request
#define CMD_ROLLBACK   0x08    // Change all EEP Regsters to Factory Default value
#define CMD_REBOOT     0x09    // Request Reboot

// Size
#define MIN_PACKET_SIZE                     7
#define MIN_ACK_PACKET_SIZE                 9
#define WRITE_PACKET_SIZE                   13
#define MAX_PACKET_SIZE                     223
#define MAX_DATA_SIZE                       (MAX_PACKET_SIZE-MIN_PACKET_SIZE)

//------------------------------------------------------------------------------

enum class servocmd : unsigned {
    eep_write,
    eep_read,
    ram_write,
    ram_read,
    i_jog,
    s_jog,
    stat,
    rollback,
    reboot,
    no_cmd
};

struct i_jog_part
{
    int16le pos;
    uint8_t mode;
    uint8_t id;
    uint8_t time;
};

struct s_jog_part
{
    int16le pos;
    uint8_t mode;
    uint8_t id;
};

struct msgPacket
{
    uint8_t buffer[MIN_PACKET_SIZE+1]; // Not its true size!

    uint16_t calcCheckSum()
    {
        uint8_t chksum1 = 0, chksum2;
        const std::size_t len = buffer[idx::packet_size];

        for (std::size_t i= 0; i < len; ++i)
        {
            chksum1 ^= buffer[i];
        }
        chksum2 = ~chksum1;
        return (chksum1 << 8 | chksum2) & 0xFEFE;
    }


    bool verifyCheckSum()
    {
        return
            ((buffer[idx::checksum1] ^ buffer[idx::checksum2]) == 0xFE)
            && ((calcCheckSum() & 0xFE) == buffer[idx::checksum1]);
    }

    msgPacket(uint8_t len, uint8_t id, uint8_t cmd)
    {
        buffer[idx::header_packet_1] = buffer[idx::header_packet_2] = HEADER;
        buffer[idx::packet_size] = len;
        buffer[idx::motor_id]    = id;
        buffer[idx::command]     = cmd;
    }
    void setCheckSum()
    {
        buffer[idx::checksum1] = 0;
        buffer[idx::checksum2] = 0;
        const auto a = calcCheckSum();
        buffer[idx::checksum1] = (a >> 8) & 0xFE;
        buffer[idx::checksum2] = a & 0xFE;
    }

    enum idx {
           header_packet_1, header_packet_2,
           packet_size,
           motor_id,
           command,
           checksum1, checksum2,
           data };
};


template <servocmd> struct msgtraits;

template <>
 struct msgtraits<servocmd::eep_write>
{
    static constexpr std::size_t len(size_t s)
    { return 2 + s + MIN_PACKET_SIZE; }
    enum { begin_off = 2 };
    enum { max_elems = 213 };
    using elem_type = uint8_t;
};

template <>
 struct msgtraits<servocmd::eep_read>
{
    static constexpr std::size_t len(size_t)
    { return 2 + MIN_PACKET_SIZE; }
};

template <>
 struct msgtraits<servocmd::ram_write> :
        msgtraits<servocmd::eep_write> {};
template <>
 struct msgtraits<servocmd::ram_read> :
        msgtraits<servocmd::eep_read> {};
template <>
 struct msgtraits<servocmd::i_jog>
{
    static constexpr std::size_t len(size_t s)
    { return s*5 + MIN_PACKET_SIZE; }

    enum { begin_off = 0 };
    enum { max_elems = 43 };
    using elem_type = i_jog_part;
};

template <>
 struct msgtraits<servocmd::s_jog>
{
    static constexpr std::size_t len(size_t s)
    { return 1+s*4 + MIN_PACKET_SIZE; }

    enum { begin_off = 1 };
    enum { max_elems = 53 };
    using elem_type = s_jog_part;
};

template <>
 struct msgtraits<servocmd::stat>
{
    static constexpr std::size_t len(size_t)
    { return MIN_PACKET_SIZE; }
};

template <>
 struct msgtraits<servocmd::rollback> :
        msgtraits<servocmd::eep_read> {};

template <>
 struct msgtraits<servocmd::reboot> :
        msgtraits<servocmd::stat> {};

template <>
 struct msgtraits<servocmd::no_cmd>
{
    static constexpr std::size_t len(size_t s)
    { return MIN_PACKET_SIZE + s; }

    enum { begin_off = 0 };
    enum { max_elems = 215 };
    using elem_type = uint8_t;
};

template <servocmd cmd>
 struct msgPacket_iterators : msgPacket
{
    msgPacket_iterators(uint8_t id, uint8_t len, uint8_t addr)
     : msgPacket(id, len, addr)   {}

    using  elem_type = typename msgtraits<cmd>::elem_type;
    enum { begin_off = msgtraits<cmd>::begin_off };

    elem_type &operator[](uint8_t i)
    {
        return begin()[i];
    }
    elem_type &at(uint8_t i)         { return *this[i]; }
    elem_type *begin()
    {
        return
            reinterpret_cast<elem_type*>(
                 this->buffer + msgPacket::data + begin_off
            );
    }
    elem_type *end()
    {
        return
            reinterpret_cast<elem_type *>(
                this->buffer + msgPacket::data
              + this->buffer[msgPacket::packet_size] - MIN_PACKET_SIZE
            );
    }
};


template <servocmd> struct msgtype;

template <>
struct msgtype<servocmd::stat> : msgPacket
{
    msgtype(uint8_t id, uint8_t)
    : msgPacket(msgtraits<servocmd::stat>::len(0), id, CMD_STAT) {}
};

template <>
struct msgtype<servocmd::reboot> : msgPacket
{
    msgtype(uint8_t id, uint8_t)
    : msgPacket(msgtraits<servocmd::reboot>::len(0), id, CMD_REBOOT) {}
};

template <>
struct msgtype<servocmd::rollback> : msgPacket
{
    msgtype(uint8_t id, uint8_t, bool idskip, bool baudskip)
    : msgPacket(msgtraits<servocmd::rollback>::len(0), id, CMD_ROLLBACK)
    {
        this->buffer[msgPacket::data+0] = idskip;
        this->buffer[msgPacket::data+1] = baudskip;
    }
};


template <>
struct msgtype<servocmd::eep_read> : msgPacket
{
    msgtype(uint8_t id, uint8_t len, uint8_t addr)
    : msgPacket(msgtraits<servocmd::eep_read>::len(0), id, CMD_EEP_READ)
    {
        this->buffer[msgPacket::data+0] = addr;
        this->buffer[msgPacket::data+1] = len;
    }
};


template <>
struct msgtype<servocmd::eep_write> : msgPacket_iterators<servocmd::eep_write>
{
    using base = msgPacket_iterators<servocmd::eep_write>;
    msgtype(uint8_t id, uint8_t len, uint8_t addr)
    : base(msgtraits<servocmd::eep_write>::len(len), id, CMD_EEP_WRITE)
    {
        this->buffer[msgPacket::data+0] = addr;
        this->buffer[msgPacket::data+1] = len;
    }
};



template <>
struct msgtype<servocmd::ram_read> : msgPacket
{
    msgtype(uint8_t id, uint8_t len, uint8_t addr)
    : msgPacket(msgtraits<servocmd::ram_read>::len(0), id, CMD_RAM_READ)
    {
        this->buffer[msgPacket::data+0] = addr;
        this->buffer[msgPacket::data+1] = len;
    }
};


template <>
struct msgtype<servocmd::ram_write> : msgPacket_iterators<servocmd::ram_write>
{
    using base = msgPacket_iterators<servocmd::ram_write>;
    msgtype(uint8_t id, uint8_t len, uint8_t addr)
    : base(msgtraits<servocmd::ram_write>::len(len), id, CMD_RAM_WRITE)
    {
        this->buffer[msgPacket::data+0] = addr;
        this->buffer[msgPacket::data+1] = len;
    }
};



template <>
struct msgtype<servocmd::i_jog> : msgPacket_iterators<servocmd::i_jog>
{
    using base = msgPacket_iterators<servocmd::i_jog>;
    msgtype(uint8_t id, uint8_t len)
    : base(msgtraits<servocmd::i_jog>::len(len), id, CMD_I_JOG)
    {
    }
};


template <>
struct msgtype<servocmd::s_jog> : msgPacket_iterators<servocmd::s_jog>
{
    using base = msgPacket_iterators<servocmd::s_jog>;
    msgtype(uint8_t id, uint8_t len)
    : base(msgtraits<servocmd::s_jog>::len(len), id, CMD_S_JOG)
    {
    }
    uint8_t &time()  { return this->buffer[msgPacket::data]; }
};

template <>
struct msgtype<servocmd::no_cmd> : msgPacket_iterators<servocmd::no_cmd>
{
    using base = msgPacket_iterators<servocmd::no_cmd>;
    msgtype(uint8_t id, uint8_t len)
    : base(msgtraits<servocmd::no_cmd>::len(len), id, 0)
    {
    }
};

using msgEEP_write  = msgtype<servocmd::eep_write>;
using msgEEP_read   = msgtype<servocmd::eep_read>;
using msgRAM_write  = msgtype<servocmd::ram_write>;
using msgRAM_read   = msgtype<servocmd::ram_read>;
using msgI_jog      = msgtype<servocmd::i_jog>;
using msgS_jog      = msgtype<servocmd::s_jog>;
using msgStat       = msgtype<servocmd::stat>;
using msgRollback   = msgtype<servocmd::rollback>;
using msgReboot     = msgtype<servocmd::reboot>;

//------------------------------------------------------------------------------
#endif // INTERNALS_HPP

