/**
  ******************************************************************************
  * @file    herkulex.cpp
  * @author  Yoonseok Pyo
  * @author  modifications by Mathieu Drapeau (mtiudaeu)
  * @author  modifications by James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-10-24
  * @brief   Low level control of Herkulex servo-motors
  * @todo    More refactoring is needed
  * @note    Here is the original copyright information about this file:
  * @verbatim
  * herkulex servo library for mbed
  *
  * Copyright (c) 2012-2013 Yoonseok Pyo, MIT License
  *
  * Permission is hereby granted, free of charge, to any person obtaining a
  * copy of this software and associated documentation files (the "Software"),
  * to deal in the Software without restriction, including without limitation
  * the rights to use, copy, modify, merge, publish, distribute, sublicense,
  * and/or sell copies of the Software, and to permit persons to whom the
  * Software is furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
  * DEALINGS IN THE SOFTWARE.
  * @endverbatim
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include "herkulex.h"
//------------------------------------------------------------------------------
#include <cstdint>
#include <algorithm>
#include <iterator>
#include "Tools.h"
#include "scope_exit.hpp"
//------------------------------------------------------------------------------

using std::size_t;

//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// This object is empty and hence cannot be modified.
static Herkulex::MsgHandler defaulthandler;


//------------------------------------------------------------------------------
struct wait_protocol : Herkulex::fwd_MsgHandler
{
    wait_protocol(MsgHandler *f)
    : fwd_MsgHandler{f} {}
    bool wait() volatile
    {
        while (waitf) {}
        return true;
    }
protected:
    volatile bool waitf = true;
};
//------------------------------------------------------------------------------
template <typename F>
struct override_stat : wait_protocol
{
    F f;

    override_stat(MsgHandler *mh, F &&f)
     : wait_protocol(mh), f(std::move(f)) {}

    virtual void stat(msgStat *m) override
    {
        bool waitb = waitf;
        f(waitb, m);
        fwd_MsgHandler::stat(m);
        waitf = waitb;
    }
};
//------------------------------------------------------------------------------
template <typename F>
struct override_read : wait_protocol
{
    F f;
    override_read(MsgHandler *mh, F &&f)
     : wait_protocol(mh), f(std::move(f)) {}

    virtual void ram_read(msgRAM_read *m) override
    {
        bool waitb = waitf;
        f(waitb, m);
        fwd_MsgHandler::ram_read(m);
        waitf = waitb;
    }
};
//------------------------------------------------------------------------------
template <typename F>
 auto make_override_read(Herkulex::MsgHandler *mh, F &&f) -> override_read<F>
{
    return {mh, std::move(f)};
}
//------------------------------------------------------------------------------
template <typename F>
 auto make_override_stat(Herkulex::MsgHandler *mh, F &&f) -> override_stat<F>
{
    return {mh, std::move(f)};
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
Herkulex::Herkulex(UARTInterface *com)
 : com{com}, mhandler{&defaulthandler}
{
    com->configure(666666);
    com->callback(this);
}
//------------------------------------------------------------------------------
Herkulex::~Herkulex()
{
    com->resetRxHandler();
}
//---------------------------------------------------------------------------
void Herkulex::sendPacket(fifo_ptr &ptr)
{
    enum { min_delay_us = 580 };
    static_cast<msgPacket *>(ptr.get())->setCheckSum();

    // It seems those motors doesn't like back-to-back packets, they error out.
    // So, if the last time we sent a command is too near from now, wait some
    // time.
    while (cpuc.cpudiff() < min_delay_us * cpuc.cycle2us) {}
    cpuc.upd();

    com->push(ptr);
}
//------------------------------------------------------------------------------
void Herkulex::send_eep_write(uint8_t id, uint8_t addr,
                              const uint8_t *beg, const uint8_t *end)
{
    constexpr auto cmd = servocmd::eep_write;

    auto msg = newMsg<cmd>(id, (end-beg), addr);

    for (auto &v : *msg)
    {
        v = *beg++;
    }
    sendPacket(msg);
}
//------------------------------------------------------------------------------
void Herkulex::send_eep_read(uint8_t id, uint8_t addr, uint8_t len)
{
    constexpr auto cmd = servocmd::eep_read;

    auto msg = newMsg<cmd>(id, len, addr);
    sendPacket(msg);
}
//------------------------------------------------------------------------------
void Herkulex::send_ram_write(uint8_t id, uint8_t addr,
                              const uint8_t *beg, const uint8_t *end)
{
    constexpr auto cmd = servocmd::ram_write;

    auto msg = newMsg<cmd>(id, (end-beg), addr);

    for (auto &v : *msg)
    {
        v = *beg++;
    }
    sendPacket(msg);
}
//------------------------------------------------------------------------------
void Herkulex::send_ram_read(uint8_t id, uint8_t addr, uint8_t len)
{
    constexpr auto cmd = servocmd::ram_read;

    auto msg = newMsg<cmd>(id, len, addr);
    sendPacket(msg);
}
//------------------------------------------------------------------------------
void Herkulex::send_i_jog(uint8_t id, const i_jog_part *beg,
                                      const i_jog_part *end)
{
    constexpr auto cmd = servocmd::i_jog;

    auto msg = newMsg<cmd>(id, (end-beg));

    for (auto &v : *msg)
    {
        v = *beg++;
    }
    sendPacket(msg);
}
//------------------------------------------------------------------------------
void Herkulex::send_s_jog(uint8_t id, uint8_t time,
                          const s_jog_part *beg, const s_jog_part *end)
{
    constexpr auto cmd = servocmd::s_jog;

    auto msg = newMsg<cmd>(id, (end-beg));

    msg->time() = time;
    for (auto &v : *msg)
    {
        v = *beg++;
    }
    sendPacket(msg);
}
//------------------------------------------------------------------------------
void Herkulex::send_stat(uint8_t id)
{
    constexpr auto cmd = servocmd::stat;

    auto msg = newMsg<cmd>(id, 0);
    sendPacket(msg);
}
//------------------------------------------------------------------------------
void Herkulex::send_rollback(uint8_t id, bool keepID, bool keepBaud)
{
    constexpr auto cmd = servocmd::rollback;

    auto msg = newMsg<cmd>(id, 0, keepID, keepBaud);
    sendPacket(msg);
}
//------------------------------------------------------------------------------
void Herkulex::send_reboot(uint8_t id)
{
    constexpr auto cmd = servocmd::reboot;

    auto msg = newMsg<cmd>(id, 0);
    sendPacket(msg);
}
//------------------------------------------------------------------------------
void Herkulex::clear(uint8_t id)
{
    send_ram_write(id, RAM_STATUS_ERROR, {0, 0});
}
//------------------------------------------------------------------------------
void Herkulex::setTorque(uint8_t id, uint8_t cmdTorque)
{
    send_ram_write(id, RAM_TORQUE_CONTROL, {cmdTorque});
}
//------------------------------------------------------------------------------
void Herkulex::positionControl(uint8_t id, uint16_t position, uint8_t playtime,
                                                                uint8_t setLED)
{
    send_s_jog(id, playtime,
               {{position, static_cast<uint8_t>(POS_MODE | setLED), id}}
               );
}

//------------------------------------------------------------------------------
void Herkulex::velocityControl(uint8_t id, int16_t speed, uint8_t setLED)
{
    send_s_jog(id, 0,
               {{speed, static_cast<uint8_t>(TURN_MODE | setLED), id}}
               );
}

//------------------------------------------------------------------------------
int8_t Herkulex::getStatus(uint8_t id)
{
    const auto old_hnd = mhandler;
    auto se = scope_exit([=]
    {
        this->mhandler = old_hnd;
    });

    uint8_t val;
    auto new_hnd = make_override_stat(old_hnd,
    [id, &val](bool &wait, msgStat *m)
    {
        if (m->buffer[msgPacket::idx::motor_id] == id)
        {
            val  = m->buffer[msgPacket::idx::data];
            wait = false;
        }
    });

    mhandler=&new_hnd;

    send_stat(id);

    new_hnd.wait();
    return val;
}

//------------------------------------------------------------------------------
int16_t Herkulex::getPos(uint8_t id)
{
    enum { len = 2 };

    const auto old_hnd = mhandler;
    auto se = scope_exit([=]
    {
        this->mhandler = old_hnd;
    });

    int16_t val = -1;

    auto new_hnd = make_override_read(old_hnd,
    [id, &val](bool &wait, msgRAM_read *m)
    {
        if (    m->buffer[msgPacket::idx::motor_id] == id
            and m->buffer[msgPacket::idx::data+0]   == RAM_CALIBRATED_POSITION
            and m->buffer[msgPacket::idx::data+1]   == len)
        {
            val  = m->buffer[msgPacket::idx::data+2]
                 + (m->buffer[msgPacket::idx::data+3] << 8);
            wait = false;
        }
    });

    mhandler=&new_hnd;

    send_ram_read(id, RAM_CALIBRATED_POSITION, len);

    new_hnd.wait();
    return val;
}
//------------------------------------------------------------------------------
 void Herkulex::handler(MsgHandler *h)
 {
     mhandler = h ? h : &defaulthandler;
 }
//------------------------------------------------------------------------------
void Herkulex::rx(int ch)
{
    // This implementation is specific for Herkulex message reception.

    // Store first
    buffer[idx] = ch;

    const auto msg     = reinterpret_cast<msgPacket *>(buffer);

    // Warning: these variable are not valid at all times, read them only when
    // there are updated.

    const uint8_t &len = buffer[msgPacket::packet_size];
    const uint8_t &cmd = buffer[msgPacket::idx::command];

    switch (idx++)
    {
        case msgPacket::idx::header_packet_1:
        case msgPacket::idx::header_packet_2:
            if (ch != HEADER)
            {
                idx = 0;
                // ERROR header
                mhandler->err_header(msg);
            }
            break;
        case msgPacket::idx::packet_size:
            if (ch == 255)
            {
                idx = 0;
                // ERROR size (considered a header error)
                mhandler->err_header(msg);
            }
            break;
        default:
            if (idx >= len)
            {
                if (msg->verifyCheckSum())
                {
                    // OK, call handlers
                    switch (cmd)
                    {
                    case CMD_EEP_READ_ACK:
                        mhandler->eep_read(static_cast<msgEEP_read *>(msg));
                        break;
                    case CMD_RAM_READ_ACK:
                        mhandler->ram_read(static_cast<msgRAM_read *>(msg));
                        break;
                    case CMD_STAT_ACK:
                        mhandler->stat(static_cast<msgStat *>(msg));
                        break;
                    default:
                        mhandler->unknownPacket(msg);
                        break;
                    }
                }
                else
                {
                    // ERROR checksum
                    mhandler->err_checksum(msg);
                }
                idx = 0;
            }
            break;
    }
}
//------------------------------------------------------------------------------
