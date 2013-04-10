#include "Tools.h"

bool Tools::Timeout( unsigned int maxDelay, const volatile bool& waitFlag )
{
    unsigned int nb1(0),nb2(1);
    while( waitFlag && nb2 < maxDelay )
    {
        int tmp = nb2;
        nb2 += nb1;
        nb1 = tmp;

        Delay(nb2);
    }

    if( nb2 > maxDelay )
    {
         return true;
    }

    return false;
}

void Tools::Delay( volatile int nCount )
{
    while(nCount--){}
}
