#ifndef TOOLS_H_INCLUDED
#define TOOLS_H_INCLUDED



namespace Tools
{
    const int DELAY_AROUND_6S = 90000000;
    const int DELAY_AROUND_1S = 15000000;

    // Return true if it timeout.
    bool Timeout( unsigned int maxDelay, const volatile bool& waitFlag );
    void Delay( volatile int nCount );

};

#endif // TOOLS_H_INCLUDED
