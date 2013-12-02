/**
  ******************************************************************************
  * @file    HerkulexMan.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-21
  * @brief   Herkulex manager (application specific)
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#ifndef HERKULEXMAN_HPP
#define HERKULEXMAN_HPP
//------------------------------------------------------------------------------
#include "herkulex.h"
//------------------------------------------------------------------------------

class HerkulexMan
{
public:

    enum
    {
            ID_R_HIP_YAW            = 1,
            ID_L_HIP_YAW            = 2,
            ID_R_HIP_ROLL           = 3,
            ID_L_HIP_ROLL           = 4,
            ID_R_HIP_PITCH          = 253,
            ID_L_HIP_PITCH          = 6,
            ID_R_KNEE               = 7,
            ID_L_KNEE               = 8,
            ID_R_ANKLE_PITCH        = 9,
            ID_L_ANKLE_PITCH        = 10,
            ID_R_ANKLE_ROLL         = 11,
            ID_L_ANKLE_ROLL         = 12,
            ID_HEAD_PAN             = 13,
            ID_HEAD_TILT            = 14,
            NUMBER_OF_JOINTS
    };

    HerkulexMan(Herkulex *h) : herk(h) {}

    Herkulex *herkulex() const { return herk; }
private:
    Herkulex *herk;
};

//------------------------------------------------------------------------------
#endif // HERKULEXMAN_HPP

