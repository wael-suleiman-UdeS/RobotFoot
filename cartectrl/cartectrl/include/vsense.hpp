/**
  ******************************************************************************
  * @file    vsense.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 1.0
  * @date    2013-11-06
  * @brief   Allow sensing of battery voltage
  ******************************************************************************
  */

#ifndef VSENSE_HPP
#define VSENSE_HPP
//------------------------------------------------------------------------------
#include <algorithm>
//------------------------------------------------------------------------------


namespace bsp
{


/**
 *  Allow sensing of battery. As a hidden feature (from the interface), when
 *  battery voltage exceeds limits set, an audible alarm occurs and LED1
 *  flashes: slow (2Hz) if it is an under-voltage situation, fast (15Hz)
 *  otherwise.
 *
 *  Please note that the voltage reading isn't reliable if the main supply
 *  exceeds 10V or is below 4.5V, approximatively.
 */
class VSense
{
public:
    VSense();
    /**
     *  @brief Set limits of voltage
     *  @param min_volt Minimum voltage
     *  @param max_volt Maximum voltage
     */
    void setLimits(float min_volt, float max_volt)
    {
        low_voltage = std::max(min_volt, 0.f);
        hi_voltage  = std::max(max_volt, low_voltage);
    }
    /**
     *  @brief Does what you expect.
     */
    virtual float getVoltage() const = 0;
protected:
    bool  alarm = false;
    float low_voltage = 0.f;
    float hi_voltage =  9999.f;
};

extern VSense &vsense;


} // namespace bsp


//------------------------------------------------------------------------------
#endif // VSENSE_HPP
