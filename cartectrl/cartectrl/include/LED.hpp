/**
  ******************************************************************************
  * @file    LED.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 1.0
  * @date    2013-09-05
  * @brief   LED control, rely on bsp::GPIO_Pin
  ******************************************************************************
  */

#ifndef LED_HPP
#define LED_HPP
//------------------------------------------------------------------------------
#include <stm32f4xx.h>
#include <cstdint>
#include "bsp/GPIO.hpp"
//------------------------------------------------------------------------------


namespace bsp
{


/** @addtogroup LED
  * @{
  */

/**
  * @addtogroup Control
  */

template <gpio g, std::uint32_t bit>
 class LED : private GPIO_Pin<g, bit>
{
    using Parent = GPIO_Pin<g, bit>;
public:
    LED()
     : Parent{pinfunction::output, outtype::pushpull,
              pullresistor::none,  iospeed::_2MHz}
    {}

    using Parent::state;
    using Parent::set;
    using Parent::reset;
    using Parent::toggle;
};

/**
  * @}
  */

/**
  * @}
  */


}



//------------------------------------------------------------------------------
#endif // LED_HPP
