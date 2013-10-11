/**
  ******************************************************************************
  * @file    miscutils.h
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version V1.0.0
  * @date    2013-04-12
  * @brief   Various utils
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#ifndef MISCUTILS_H
#define MISCUTILS_H
//------------------------------------------------------------------------------


// Request an alignment for an object.
// Do like this: Type n ALIGNED(4);
#define ALIGNED(x) __attribute__ ((aligned (x)))


//------------------------------------------------------------------------------
#endif // MISCUTILS_H
