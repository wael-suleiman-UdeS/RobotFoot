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
#include "Tools.h"

#include <stm32f4xx.h>
#include <misc.h>


namespace{

#define MAX_LEN 20 // FIXME : Maximum msg length (WHY 20?? Check documentation)
volatile uint8_t receivedMsg[MAX_LEN]; // RX msg
volatile bool waitingForData = false;

}

using std::size_t;

struct msgPacket_base
{
    static
    uint16_t calcCheckSum(uint8_t const *const buf, std::size_t len)
    {
        uint8_t chksum1 = 0, chksum2;
        for (size_t i= 0; i < len; ++i)
        {
            chksum1 ^= buf[i];
        }
        chksum2 = ~chksum1;
        return (chksum1 << 8 | chksum2) & 0xFEFE;
    }
    static
    uint16_t calcCheckSum(uint8_t *const buf, std::size_t len, uint8_t *res)
    {
        auto r = calcCheckSum(buf, len);
        *res++ = (r >> 8) & 0xFF;
        *res   = r & 0xFF;
        return r;
    }
    static
    bool verifyCheckSum(uint8_t const *const buf, std::size_t len)
    {
        return (buf[idx::checksum1] & 0xFE) == (~buf[idx::checksum2] & 0xFE)
            && (calcCheckSum(buf, len) & 0xFE) == buf[idx::checksum1];
    }
    enum idx {
           header_packet_1, header_packet_2,
           packet_size,
           motor_id,
           command,
           checksum1, checksum2,
           data };
};

template <size_t L>
 struct msgPacket : msgPacket_base
{
    static_assert(L >= MIN_PACKET_SIZE, "Packet size is too small.");
    typedef msgPacket_base Parent;
    uint8_t buffer[L] = {HEADER, HEADER};
    msgPacket(uint8_t id, uint8_t cmd)
    {
        buffer[idx::packet_size] = L;
        buffer[idx::motor_id]    = id;
        buffer[idx::command]     = cmd;
    }
    void calcCheckSum()
    {
        std::fill_n(buffer + idx::checksum1, 2, 0);
        Parent::calcCheckSum(buffer, L, buffer + idx::checksum1);
    }
};

//------------------------------------------------------------------------------
Herkulex::Herkulex(/*PinName tx, PinName rx, uint32_t baudRate*/)
{
}

//------------------------------------------------------------------------------
Herkulex::~Herkulex()
{
}
//------------------------------------------------------------------------------
Herkulex* Herkulex::GetInstance()
{
    static Herkulex instance[1] = {{}};
    return instance;
}

//------------------------------------------------------------------------------
void Herkulex::clear(uint8_t id)
{
    msgPacket<11> txBuf(id, CMD_RAM_WRITE);

    txBuf.buffer[7] = RAM_STATUS_ERROR;    // Address 48DEBUG
    txBuf.buffer[8] = BYTE2;               // Length
    txBuf.buffer[9] = 0;                   // Clear RAM_STATUS_ERROR
    txBuf.buffer[10]= 0;                   // Clear RAM_STATUS_DETAIL

    txPacket(txBuf.buffer);
}
//------------------------------------------------------------------------------
void Herkulex::reboot(uint8_t id)
{
    msgPacket<MIN_PACKET_SIZE> txBuf(id, CMD_REBOOT);
    txPacket(txBuf.buffer);
}
//------------------------------------------------------------------------------
void Herkulex::setTorque(uint8_t id, uint8_t cmdTorque)
{
    msgPacket<10> txBuf(id, CMD_RAM_WRITE);

    txBuf.buffer[7] = RAM_TORQUE_CONTROL;  // Address 52
    txBuf.buffer[8] = BYTE1;               // Length
    txBuf.buffer[9] = cmdTorque;           // Torque ON

    txPacket(txBuf.buffer);
}

//------------------------------------------------------------------------------
void Herkulex::positionControl(uint8_t id, uint16_t position, uint8_t playtime,
                                                                uint8_t setLED)
{
    if (position > 1023) return;

    msgPacket<12> txBuf(id, CMD_S_JOG);

    txBuf.buffer[7]  = playtime;               // Playtime
    txBuf.buffer[8]  = position & 0x00FF;      // Position (LSB, Least Significant Bit)
    txBuf.buffer[9]  =(position & 0xFF00) >> 8;// position (MSB, Most Significanct Bit)
    txBuf.buffer[10] = POS_MODE | setLED;      // Pos Mode and LED on/off
    txBuf.buffer[11] = id;                     // Servo ID

    txPacket(txBuf.buffer);
}

//------------------------------------------------------------------------------
void Herkulex::velocityControl(uint8_t id, int16_t speed, uint8_t setLED)
{
    if (speed > 1023 || speed < -1023) return;

    msgPacket<12> txBuf(id, CMD_S_JOG);

    txBuf.buffer[7]  = 0;                      // Playtime, unmeaningful in turn mode
    txBuf.buffer[8]  = speed & 0x00FF;         // Speed (LSB, Least Significant Bit)
    txBuf.buffer[9]  =(speed & 0xFF00) >> 8;   // Speed (MSB, Most Significanct Bit)
    txBuf.buffer[10] = TURN_MODE | setLED;     // Turn Mode and LED on/off
    txBuf.buffer[11] = id;                     // Servo ID

    txPacket(txBuf.buffer);
}

//------------------------------------------------------------------------------
int8_t Herkulex::getStatus(uint8_t id)
{
    uint8_t status;
    msgPacket<7> txBuf(id, CMD_STAT);

    txPacket(txBuf.buffer);

    uint8_t rxBuf[9];
    if( !rxPacket(9, rxBuf) ) return -1;


    // Checksum1
    uint8_t chksum1 = (rxBuf[2]^rxBuf[3]^rxBuf[4]^rxBuf[7]^rxBuf[8]) & 0xFE;
    if (chksum1 != rxBuf[5])
    {
        return -1;
    }

    // Checksum2
    uint8_t chksum2 = (~rxBuf[5]&0xFE);
    if (chksum2 != rxBuf[6])
    {
        return -1;
    }

    status = rxBuf[7];  // Status Error
  //status = rxBuf[8];  // Status Detail

    return status;
}

//------------------------------------------------------------------------------
int16_t Herkulex::getPos(uint8_t id)
{
    uint16_t position = 0;

    msgPacket<9> txBuf(id, CMD_RAM_READ);

    txBuf.buffer[7] = RAM_CALIBRATED_POSITION; // Address 52
    txBuf.buffer[8] = BYTE2;                   // Address 52 and 53

    txPacket(txBuf.buffer);

    uint8_t rxBuf[13];
    if( !rxPacket(13, rxBuf) ) return -1;

    // Checksum1
    uint8_t chksum1 = (rxBuf[2]^rxBuf[3]^rxBuf[4]^rxBuf[7]^rxBuf[8]^rxBuf[9]^rxBuf[10]^rxBuf[11]^rxBuf[12]) & 0xFE;
    if (chksum1 != rxBuf[5])
    {
        return -1;
    }

    // Checksum2
    uint8_t chksum2 = (~rxBuf[5]&0xFE);
    if (chksum2 != rxBuf[6])
    {
        return -1;
    }

    position = ((rxBuf[10]&0x03)<<8) | rxBuf[9];

    return position;
}

//------------------------------------------------------------------------------
void Herkulex::txPacket(uint8_t packetSize, uint8_t* data)
{
    // To change.
    //txBuf.calcCheckSum();

    for(uint8_t i = 0; i < packetSize ; i++)
    {
        while ( !USART_GetFlagStatus(USART3, USART_FLAG_TC) );
        USART_SendData(USART3, *data++);
    }
}

//------------------------------------------------------------------------------
bool Herkulex::rxPacket(uint8_t packetSize, uint8_t* data)
{
    waitingForData = true;

    // TODO : Timeout time depends on baudrate
    if ( Tools::Timeout( 15000, waitingForData ) )
    {
        return false;
    }
    std::copy_n(receivedMsg, packetSize, data);

    return true;
}


//------------------------------------------------------------------------------

// This is the interrupt request handler (IRQ) for ALL USART3 interrupts
extern "C" void USART3_IRQHandler(void){

	// Check if the USART3 receive interrupt flag was set
	if( USART_GetITStatus(USART3, USART_IT_RXNE) ){

        // This implementation is specific for Herkulex message reception.
        bool status = true;
		static uint8_t cnt = 0;
		static uint8_t msgLength = MAX_LEN;
		uint8_t data = USART3->DR; // the character from the USART3 data register is saved in t

        // Check is first two data are HEADER
        if( cnt == 0 || cnt == 1 )
        {
            if( data != HEADER ){ //FIXME : Should trigger an error!
                status = false;
            }
        }
        // Check the msg length
        else if( cnt == 2 )
        {
            if( data > MAX_LEN ){ //FIXME : Should trigger an error!
                status = false;
            }
            else
                msgLength = data;
        }

        // Fill the msg buffer
		if( status && cnt < msgLength - 1 ){
			receivedMsg[cnt] = data;
			cnt++;
		}
		// Reset cnt
		else{
		    if( status ){
		        // If status is fine, we fill the buffer with last data before reset
                receivedMsg[cnt] = data;
		    }
			cnt = 0;
			waitingForData = false;
		}
	}
}
