/**
  ******************************************************************************
  * @file    GPIO.cpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-10-21
  * @brief   Easy configuration and control of GPIOs
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include "bsp/GPIO.hpp"
//------------------------------------------------------------------------------
#include <cstdlib>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
//------------------------------------------------------------------------------

using std::uint32_t;
using std::uint16_t;
using std::uint8_t;

using bsp::iospeed;
using bsp::outtype;
using bsp::pullresistor;
using bsp::pinfunction;

#define CONCAT2(x, y)   x ## y
#define CONCAT(x,y)     CONCAT2(x,y)

#define CONVERTER(tenum, otype, ...) \
static otype CONCAT(tenum,_conv) (tenum var) \
{ \
    switch (var) \
    { \
    __VA_ARGS__ \
    default: std::abort(); \
    } \
}

#define CCASE(val1, val2)   case val1: return val2;

/**
  * These macros define conversion functions used in @c init. Macro are evil,
  * but still useful to eliminiate boiler plate code.
  */
CONVERTER(iospeed, GPIOSpeed_TypeDef,
          CCASE(iospeed::_2MHz,   GPIO_Speed_2MHz)
          CCASE(iospeed::_25MHz,  GPIO_Speed_25MHz)
          CCASE(iospeed::_50MHz,  GPIO_Speed_50MHz)
          CCASE(iospeed::_100MHz, GPIO_Speed_100MHz)
          )

CONVERTER(outtype, GPIOOType_TypeDef,
          CCASE(outtype::pushpull,  GPIO_OType_PP)
          CCASE(outtype::opendrain, GPIO_OType_OD)
          )

CONVERTER(pullresistor, GPIOPuPd_TypeDef,
          CCASE(pullresistor::none,     GPIO_PuPd_NOPULL)
          CCASE(pullresistor::pullup,   GPIO_PuPd_UP)
          CCASE(pullresistor::pulldown, GPIO_PuPd_DOWN)
          )

CONVERTER(pinfunction, GPIOMode_TypeDef,
          CCASE(pinfunction::input,     GPIO_Mode_IN)
          CCASE(pinfunction::output,    GPIO_Mode_OUT)
          CCASE(pinfunction::alternate, GPIO_Mode_AF)
          CCASE(pinfunction::analog,    GPIO_Mode_AN)
          )

#undef CCASE
#undef CONVERTER
#undef CONCAT
#undef CONCAT2

#define GPIO_(x)    { RCC_AHB1Periph_GPIO##x, GPIO##x }

// This associate gpios with their corresponding enable bit.
static struct {
    uint32_t         powerbit;
    GPIO_TypeDef    *gpio;
} const GPIOconf[] =
{
    GPIO_(A), GPIO_(B), GPIO_(C), GPIO_(D),
    GPIO_(E), GPIO_(F), GPIO_(G), GPIO_(H),
    GPIO_(I)
};

#undef GPIO_

namespace bsp
{


/**
  * @brief  ctor
  * @param  g    GPIO to use
  * @param  bit_ Pin to use
  */
GPIO_Pin_base::GPIO_Pin_base(gpio g, uint16_t bit_)
 : pgpio_{GPIOconf[static_cast<unsigned>(g)].gpio}, bit_{bit_}
{
    imp_init(g);
}
/**
  * @brief  Configure pin
  * @param  fun   Pin function
  * @param  ot    Output mode
  * @param  pr    Pull-up/down configuration
  * @param  speed Pin speed
  * @return None
  */
void GPIO_Pin_base::init(pinfunction fun, outtype ot,
                         pullresistor pr, iospeed speed)
{
    GPIO_InitTypeDef gpio[1];
    gpio->GPIO_Pin   = (1u << bit_);
    gpio->GPIO_Mode  = pinfunction_conv(fun);
    gpio->GPIO_Speed = iospeed_conv(speed);
    gpio->GPIO_OType = outtype_conv(ot);
    gpio->GPIO_PuPd  = pullresistor_conv(pr);

    GPIO_Init(pgpio_, gpio);
}
/**
  * @brief  Activate the gpio used by the pin
  * @param  b state or None
  * @return None
  * @note   Only called by ctor
  */
void GPIO_Pin_base::imp_init(gpio t)
{
    RCC_AHB1PeriphClockCmd(GPIOconf[static_cast<unsigned>(t)].powerbit, ENABLE);
}
//------------------------------------------------------------------------------



} // namespace bsp
