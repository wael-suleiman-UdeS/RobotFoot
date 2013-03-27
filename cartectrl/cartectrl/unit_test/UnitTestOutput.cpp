#include "UnitTestOutput.h"

#include <stm32f4xx_usart.h>

void UnitTestOutput::SendMessage( const char* data )
{
    if( !data )
        return;

    while ( *data != '\0' )
    {
        while( !(USART2->SR & 0x00000040) );
                USART_SendData(USART2, *data );
        data++;
    }
}
