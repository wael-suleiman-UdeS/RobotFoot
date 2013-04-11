/**
  ******************************************************************************
  * @file    usb_com.h
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version V1.0.0
  * @date    2013-04-10
  * @brief   Allow communication through usb cdc
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#ifndef USB_COM_H
#define USB_COM_H
//------------------------------------------------------------------------------
#include <stdint.h>
#include <vector>
#include <string>

#if __GXX_EXPERIMENTAL_CXX0X__ == 1 /* require c++11 support */
 #include <array>
#endif
//------------------------------------------------------------------------------

/** @addtogroup Peripheral
  * @{
  */

/** @addtogroup USB
  * @{
  */

namespace usb
{

using std::vector;
using std::string;

#if __GXX_EXPERIMENTAL_CXX0X__ == 1 /* require c++11 support */
 using std::array;
#endif


int init();

// Base trasmit function
int write(const uint8_t *m, size_t s);

// Convenience functions:

// All the convenience functions are based on that one. Pass simple pointers to
// object type in it, or you'll get pretty Bad Results™.
// Perfect with pointer to char; okay with pointer to integer, though you should
// mind the endianness of both ends of the communication. You might use a POD
// with no pointer member. Anything else is just plain silly.
template <typename T>
 inline int write(const T *m, size_t s)
{
    return write(reinterpret_cast<const uint8_t *>(m), s*sizeof(T));
}

// Adaptor: any vector (see comment above for restrictions on type T).
template <typename T>
 inline int write(const vector<T> &v)
{
    return write(v.data(), v.size());
}
// Adaptor: a string.
inline int write(const string &s)
{
    return write(s.data(), s.length());
}

// Adaptor: any plain array (see comment above for restrictions on type T).
template <typename T, size_t N>
 inline int write(const T (&arr)[N])
{
    return write(arr, N);
}

#if __GXX_EXPERIMENTAL_CXX0X__ == 1 /* require c++11 support */

// Adaptor: any std::array (see comment above for restrictions on type T).
template <typename T, size_t N>
 inline int write(const array<T, N> &arr)
{
    return write(arr.data(), N);
}

#endif

// Base trasmit function
int read(uint8_t *m, size_t s);

// Convenience functions:

// All the convenience functions are based on that one. Pass simple pointers to
// object type in it, or you'll get pretty Bad Results™, ESPECIALLY more that
// one that the write overload above. The same comment does apply.
template <typename T>
 inline int read(T *m, size_t s)
{
    return read(reinterpret_cast<uint8_t *>(m), s*sizeof(T));
}

// Adaptor: any vector (see comment above for restrictions on type T).
template <typename T>
 inline int read(vector<T> &v)
{
    // TODO: Do it better!
    return read(v.data(), v.size());
}
// Adaptor: a string.
inline int read(string &s)
{
    // TODO: Do it better!
    return read(&*s.begin(), s.length());
}

// Adaptor: any plain array (see comment above for restrictions on type T).
template <typename T, size_t N>
 inline int read(T (&arr)[N])
{
    return read(arr, N);
}

#if __GXX_EXPERIMENTAL_CXX0X__ == 1 /* require c++11 support */

// Adaptor: any std::array (see comment above for restrictions on type T).
template <typename T, size_t N>
 inline int read(array<T, N> &arr)
{
    return read(arr.data(), N);
}

#endif

}

/**
  * @}
  */

/**
  * @}
  */
//------------------------------------------------------------------------------
#endif // USB_COM_H

