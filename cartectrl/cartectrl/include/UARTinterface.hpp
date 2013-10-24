/**
  ******************************************************************************
  * @file    UARTinterface.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-18
  * @brief   UART interface classes
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#ifndef UARTINTERFACE_HPP
#define UARTINTERFACE_HPP
//------------------------------------------------------------------------------
#include "FIFOalloc.hpp"
//------------------------------------------------------------------------------


struct UART_Callback
{
    virtual ~UART_Callback() {}
    virtual void rx(int) = 0;
    virtual void xferd() {}
};


class UARTInterface
{
public:
    UARTInterface(UART_Callback *newhandler = nullptr);

    virtual fifo_ptr alloc(size_t) = 0;

    void push(fifo_ptr &ptr)
    {
        ptr.release();
        ++pushcnt;
        notify();
    }
    void push(fifo_ptr &&ptr)
    {
        return push(ptr);
    }

    virtual int configure(unsigned baud) = 0;

    unsigned numPackets() const volatile { return pushcnt - popdcnt; }

    UART_Callback  *callback() const { return cb; }
    void            callback(UART_Callback *newcb);

    void resetRxHandler(UART_Callback *newcb = nullptr)
    {
        return callback(newcb);
    }
protected:
    virtual void notify() = 0;

    void updpopcnt() { ++popdcnt; }
private:
    unsigned pushcnt = 0;
    unsigned popdcnt = 0;

    UART_Callback *cb;
};


//------------------------------------------------------------------------------
#endif // UARTINTERFACE_HPP
