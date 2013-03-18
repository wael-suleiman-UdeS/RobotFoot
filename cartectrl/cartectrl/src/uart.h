#ifndef UART_H
#define UART_H

#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src

namespace UARTUtility
{
    void init_USART1(uint32_t baudrate);
    //void USART_puts(USART_TypeDef* USARTx, const volatile char *s);

    // Temporary test function
    void SendTest(USART_TypeDef* USARTx, uint8_t packetSize, uint8_t *s);

}

#endif // UART_H
