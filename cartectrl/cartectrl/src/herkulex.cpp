//------------------------------------------------------------------------------
/* herkulex servo library for mbed
 *
 * Copyright (c) 2012-2013 Yoonseok Pyo, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
//------------------------------------------------------------------------------
#include "herkulex.h"

#include "Tools.h"

#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <misc.h>


namespace{

#define MAX_LEN 20 // FIXME : Maximum msg length (WHY 20?? Check documentation)
volatile uint8_t receivedMsg[MAX_LEN]; // RX msg
volatile bool waitingForData = false;

}

//------------------------------------------------------------------------------
Herkulex::Herkulex(/*PinName tx, PinName rx, uint32_t baudRate*/)
{
    init_USART1(115200); // initialize USART1 @ 115200 baud
}

//------------------------------------------------------------------------------
Herkulex::~Herkulex()
{
}

//------------------------------------------------------------------------------
void Herkulex::clear(uint8_t id)
{
    uint8_t txBuf[11];

    txBuf[0] = HEADER;              // Packet Header (0xFF)
    txBuf[1] = HEADER;              // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 4; // Packet Size
    txBuf[3] = id;                  // Servo ID
    txBuf[4] = CMD_RAM_WRITE;       // Command Ram Write (0x03)
    txBuf[5] = 0;                   // Checksum1
    txBuf[6] = 0;                   // Checksum2
    txBuf[7] = RAM_STATUS_ERROR;    // Address 48DEBUG
    txBuf[8] = BYTE2;               // Length
    txBuf[9] = 0;                   // Clear RAM_STATUS_ERROR
    txBuf[10]= 0;                   // Clear RAM_STATUS_DETAIL

    // Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
    // Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]^txBuf[10]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    txPacket(11, txBuf);
}

//------------------------------------------------------------------------------
void Herkulex::setTorque(uint8_t id, uint8_t cmdTorue)
{
    uint8_t txBuf[10];

    txBuf[0] = HEADER;              // Packet Header (0xFF)
    txBuf[1] = HEADER;              // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 3; // Packet Size
    txBuf[3] = id;                  // Servo ID
    txBuf[4] = CMD_RAM_WRITE;       // Command Ram Write (0x03)
    txBuf[5] = 0;                   // Checksum1
    txBuf[6] = 0;                   // Checksum2
    txBuf[7] = RAM_TORQUE_CONTROL;  // Address 52
    txBuf[8] = BYTE1;               // Length
    txBuf[9] = cmdTorue;            // Torque ON

    // Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
    // Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    // send packet (mbed -> herkulex)
    txPacket(10, txBuf);
}

//------------------------------------------------------------------------------
void Herkulex::positionControl(uint8_t id, uint16_t position, uint8_t playtime, uint8_t setLED)
{
    if (position > 1023) return;
    if (playtime > 255) return;

    uint8_t txBuf[12];

    txBuf[0]  = HEADER;                 // Packet Header (0xFF)
    txBuf[1]  = HEADER;                 // Packet Header (0xFF)
    txBuf[2]  = MIN_PACKET_SIZE + 5;    // Packet Size
    txBuf[3]  = id;                // pID is total number of servos in the network (0 ~ 253)
    txBuf[4]  = CMD_S_JOG;              // Command S JOG (0x06)
    txBuf[5]  = 0;                      // Checksum1
    txBuf[6]  = 0;                      // Checksum2
    txBuf[7]  = playtime;               // Playtime
    txBuf[8]  = position & 0x00FF;      // Position (LSB, Least Significant Bit)
    txBuf[9]  =(position & 0xFF00) >> 8;// position (MSB, Most Significanct Bit)
    txBuf[10] = POS_MODE | setLED;      // Pos Mode and LED on/off
    txBuf[11] = id;                     // Servo ID

    // Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
    // Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]^txBuf[10]^txBuf[11]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    // send packet (mbed -> herkulex)
    txPacket(12, txBuf);
}

//------------------------------------------------------------------------------
void Herkulex::velocityControl(uint8_t id, int16_t speed, uint8_t setLED)
{
    if (speed > 1023 || speed < -1023) return;

    uint8_t txBuf[12];

    txBuf[0]  = HEADER;                 // Packet Header (0xFF)
    txBuf[1]  = HEADER;                 // Packet Header (0xFF)
    txBuf[2]  = MIN_PACKET_SIZE + 5;    // Packet Size
    txBuf[3]  = id;                // pID is total number of servos in the network (0 ~ 253)
    txBuf[4]  = CMD_S_JOG;              // Command S JOG (0x06)
    txBuf[5]  = 0;                      // Checksum1
    txBuf[6]  = 0;                      // Checksum2
    txBuf[7]  = 0;                      // Playtime, unmeaningful in turn mode
    txBuf[8]  = speed & 0x00FF;         // Speed (LSB, Least Significant Bit)
    txBuf[9]  =(speed & 0xFF00) >> 8;   // Speed (MSB, Most Significanct Bit)
    txBuf[10] = TURN_MODE | setLED;     // Turn Mode and LED on/off
    txBuf[11] = id;                     // Servo ID

    // Checksum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
    // Checksum2 = (~Checksum1)&0xFE
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]^txBuf[9]^txBuf[10]^txBuf[11]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    // send packet (mbed -> herkulex)
    txPacket(12, txBuf);
}

//------------------------------------------------------------------------------
int8_t Herkulex::getStatus(uint8_t id)
{
    uint8_t status;
    uint8_t txBuf[7];

    txBuf[0] = HEADER;                  // Packet Header (0xFF)
    txBuf[1] = HEADER;                  // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE;         // Packet Size
    txBuf[3] = id;                      // Servo ID
    txBuf[4] = CMD_STAT;                // Status Error, Status Detail request

    // Check Sum1 and Check Sum2
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    // send packet (mbed -> herkulex)
    txPacket(7, txBuf);

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

    uint8_t txBuf[9];

    txBuf[0] = HEADER;                  // Packet Header (0xFF)
    txBuf[1] = HEADER;                  // Packet Header (0xFF)
    txBuf[2] = MIN_PACKET_SIZE + 2;     // Packet Size
    txBuf[3] = id;                      // Servo ID
    txBuf[4] = CMD_RAM_READ;            // Status Error, Status Detail request
    txBuf[5] = 0;                       // Checksum1
    txBuf[6] = 0;                       // Checksum2
    txBuf[7] = RAM_CALIBRATED_POSITION; // Address 52
    txBuf[8] = BYTE2;                   // Address 52 and 53

    // Check Sum1 and Check Sum2
    txBuf[5] = (txBuf[2]^txBuf[3]^txBuf[4]^txBuf[7]^txBuf[8]) & 0xFE;
    txBuf[6] = (~txBuf[5])&0xFE;

    // send packet (mbed -> herkulex)
    txPacket(9, txBuf);

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
    for(uint8_t i = 0; i < packetSize ; i++)
    {
        while( !(USART1->SR & 0x00000040) );
            USART_SendData(USART1, *data);
		*data++;
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

    for (uint8_t i=0; i < packetSize; i++)
    {
        data[i] = receivedMsg[i];
    }
    return true;
}

//------------------------------------------------------------------------------

/* This funcion initializes the USART1 peripheral
 *
 * Arguments: baudrate --> the baudrate at which the USART is
 * 						   supposed to operate
 */
void Herkulex::init_USART1(uint32_t baudrate){

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

//------------------------------------------------------------------------------

namespace{

// This is the interrupt request handler (IRQ) for ALL USART1 interrupts
extern "C" void USART1_IRQHandler(void){

	// Check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){

        // This implementation is specific for Herkulex message reception.
        bool status = true;
		static uint8_t cnt = 0;
		static uint8_t msgLength = MAX_LEN;
		uint8_t data = USART1->DR; // the character from the USART1 data register is saved in t

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

}
