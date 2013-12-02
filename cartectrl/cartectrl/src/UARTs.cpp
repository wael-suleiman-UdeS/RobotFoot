/**
  ******************************************************************************
  * @file    UARTs.cpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-16
  * @brief   Global UART objects.
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include "UARTs.hpp"
//------------------------------------------------------------------------------
#include <stm32f4xx_usart.h>
#include "bsp/GPIO.hpp"
//------------------------------------------------------------------------------

using namespace bsp;

//------------------------------------------------------------------------------

// Prevent optimisations of reads of USARTx->DR
volatile int dummy_read;

struct XFers
{
    uint8_t  *start;
    uint16_t len;
};

static XFers UART2_Xfer, UART3_Xfer;


class UART_STM32_Common : public UARTInterface
{
public:
    using UARTInterface::callback;

    UART_STM32_Common(USART_TypeDef *uart, XFers *xfers)
     : uart{uart}, xfers{xfers} {}

    void nextXfer()
    {
        updpopcnt();
        getFIFO().pop();
        if (numPackets())
        {
            auto reg = getFIFO().access();
            xfers->start = reg.data;
            xfers->len   = reg.len;
        }
        else
        {
            USART_ITConfig(uart, USART_IT_TXE, DISABLE);
        }
    }

protected:
    USART_TypeDef *const uart;

    void init(unsigned baudrate=115200)
    {
        configure(baudrate);
        USART_Cmd(uart, ENABLE);
    }

    virtual int configure(unsigned baudrate) override
    {
        USART_InitTypeDef uartcfg[1] = {{}};

        uartcfg->USART_BaudRate = baudrate;
        uartcfg->USART_WordLength = USART_WordLength_8b;
        uartcfg->USART_StopBits = USART_StopBits_1;
        uartcfg->USART_Parity = USART_Parity_No;
        uartcfg->USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        uartcfg->USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

        USART_Init(uart, uartcfg);

        USART_ITConfig(uart, USART_IT_RXNE, ENABLE);

        return 0;
    }

    virtual void notify() override
    {
        if (!(uart->CR1 & 0x80))
        {
            auto reg = getFIFO().access();
            xfers->start = reg.data;
            xfers->len   = reg.len;
            USART_ITConfig(uart, USART_IT_TXE, ENABLE);
        }
    }

    virtual FIFOallocator &getFIFO() = 0;

    XFers *const xfers;
};
//------------------------------------------------------------------------------
class UART2_Interface_imp : public UART_STM32_Common,
                            private FIFOalloc<512>
{
public:
    UART2_Interface_imp()
     : UART_STM32_Common(USART2, &UART2_Xfer)
    {
        GPIO_Pin<gpio::a, 2>
            {pinfunction::alternate, outtype::pushpull,
             pullresistor::pullup};
        GPIO_Pin<gpio::a, 3>
            {pinfunction::alternate, outtype::pushpull,
             pullresistor::pullup};

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

        NVIC_InitTypeDef  nviccfg[1] = {{}};

        nviccfg->NVIC_IRQChannel = USART2_IRQn;
        nviccfg->NVIC_IRQChannelPreemptionPriority = 7;
        nviccfg->NVIC_IRQChannelSubPriority = 7;
        nviccfg->NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(nviccfg);

        init();
    }

    virtual fifo_ptr alloc(size_t s) override
    {
        return {*this, s};
    }

    virtual FIFOallocator &getFIFO() override
    {
        return *this;
    }
};
//------------------------------------------------------------------------------
class UART3_Interface_imp : public UART_STM32_Common,
                            private FIFOalloc<512>
{
public:
    UART3_Interface_imp()
     : UART_STM32_Common(USART3, &UART3_Xfer)
    {
        GPIO_Pin<gpio::b, 10>
            {pinfunction::alternate, outtype::pushpull,
             pullresistor::pullup};
        GPIO_Pin<gpio::b, 11>
            {pinfunction::alternate, outtype::pushpull,
             pullresistor::pullup};

        GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

        NVIC_InitTypeDef  nviccfg[1] = {{}};

        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
        nviccfg->NVIC_IRQChannel = USART3_IRQn;
        nviccfg->NVIC_IRQChannelPreemptionPriority = 0;
        nviccfg->NVIC_IRQChannelSubPriority = 1;
        nviccfg->NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(nviccfg);

        init();
    }

    virtual fifo_ptr alloc(size_t s) override
    {
        return {*this, s};
    }

    virtual FIFOallocator &getFIFO() override
    {
        return *this;
    }
};
//------------------------------------------------------------------------------
UART2_Interface_imp UART2i_imp;
UART3_Interface_imp UART3i_imp;
//------------------------------------------------------------------------------
UARTInterface &UART2i = UART2i_imp;
UARTInterface &UART3i = UART3i_imp;

/**
 *  @brief IRQ handler for the USART2 (used for RX & TX)
 */
extern "C" void USART2_IRQHandler(void)
{
    if ( USART_GetITStatus(USART2, USART_IT_ORE_RX) )
    {
        // Do a read to acknowledge!!
        dummy_read = USART2->DR;
    }
	if ( USART_GetITStatus(USART2, USART_IT_RXNE) )
	{
        UART2i_imp.callback()->rx(USART2->DR);
	}
	if ( USART_GetITStatus(USART2, USART_IT_TXE))
	{
	    if (!UART2_Xfer.len)
	    {
	        UART2i_imp.callback()->xferd();
	        UART2i_imp.nextXfer();
	    }
	    else
	    {
	        USART2->DR = *UART2_Xfer.start++;
	        UART2_Xfer.len--;
	    }
	}
}
/**
 *  @brief IRQ handler for the USART3 (used for RX & TX)
 */
extern "C" void USART3_IRQHandler(void)
{
    if ( USART_GetITStatus(USART3, USART_IT_ORE_RX) )
    {
        // Do a read to acknowledge!!
        dummy_read = USART3->DR;
    }
	if ( USART_GetITStatus(USART3, USART_IT_RXNE) )
	{
        UART3i_imp.callback()->rx(USART3->DR);
	}
	if ( USART_GetITStatus(USART3, USART_IT_TXE))
	{
	    if (!UART3_Xfer.len)
	    {
	        UART3i_imp.callback()->xferd();
	        UART3i_imp.nextXfer();
	    }
	    else
	    {
	        USART3->DR = *UART3_Xfer.start++;
	        UART3_Xfer.len--;
	    }
	}
}
//------------------------------------------------------------------------------
