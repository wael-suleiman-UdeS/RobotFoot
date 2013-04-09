#include "UnitTestOutput.h"

#include <stm32f4xx_usart.h>

#include <stdio.h>

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

void UnitTestOutput::SendMessage( int value )
{
    // Max string length for an int is around 16
    const int MaxLength = 20;
    char data[MaxLength];

    sprintf( data,"%d",value );

    SendMessage( data );
}
