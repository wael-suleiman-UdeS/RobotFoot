#ifndef TOOLS_H_INCLUDED
#define TOOLS_H_INCLUDED

class Tools
{
public:
    Tools(){}
    ~Tools(){}

    static void Timeout( unsigned int maxDelay, const volatile bool& waitFlag )
    {
        unsigned int nb1(0),nb2(1);
        while( waitFlag && nb2 < maxDelay )
        {
            int tmp = nb2;
            nb2 += nb1;
            nb1 = tmp;

            Delay(nb2);
        }
    }

    static void Delay( volatile uint32_t nCount )
    {
        while(nCount--){}
    }

};

#endif // TOOLS_H_INCLUDED
