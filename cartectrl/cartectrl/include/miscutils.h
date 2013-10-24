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


#ifdef __cplusplus

//------------------------------------------------------------------------------
#include <utility>
#include <cstdint>
//------------------------------------------------------------------------------

template <typename T>
 struct compound_assign
{
    template <typename U>
     T &operator+=(U &&u) { *this = *this+std::forward<U>(u); return *this; }
    template <typename U>
     T &operator-=(U &&u) { *this = *this-std::forward<U>(u); return *this; }
    template <typename U>
     T &operator*=(U &&u) { *this = *this*std::forward<U>(u); return *this; }
    template <typename U>
     T &operator/=(U &&u) { *this = *this/std::forward<U>(u); return *this; }
    template <typename U>
     T &operator%=(U &&u) { *this = *this%std::forward<U>(u); return *this; }
    template <typename U>
     T &operator<<=(U &&u) { *this = *this<<std::forward<U>(u); return *this; }
    template <typename U>
     T &operator>>=(U &&u) { *this = *this>>std::forward<U>(u); return *this; }
    template <typename U>
     T &operator|=(U &&u) { *this = *this|std::forward<U>(u); return *this; }
    template <typename U>
     T &operator&=(U &&u) { *this = *this&std::forward<U>(u); return *this; }
    template <typename U>
     T &operator^=(U &&u) { *this = *this^std::forward<U>(u); return *this; }
    T &operator++()   { return *this+=1; }
    T &operator--()   { return *this+=1; }
    T operator++(int) { T t = *this; ++*this; return t; }
    T operator--(int) { T t = *this; --*this; return t; }
};

// Integer that allow portable unaligned access transparently
struct int16le : compound_assign<int16le>
{
    uint8_t bytes[2];
    int16le(int16_t b=0)
        { bytes[0] = b & 0xFF; bytes[1] = (b>>8) & 0xFF; }
    operator int16_t () const
        { return bytes[0] | (bytes[1] << 8); }
};

struct uint16le : compound_assign<uint16le>
{
    uint8_t bytes[2];
    uint16le(uint16_t b=0)
        { bytes[0] = b & 0xFF; bytes[1] = (b>>8) & 0xFF; }
    operator uint16_t () const
        { return bytes[0] | (bytes[1] << 8); }
};


template <typename> struct funcref;

template <typename,typename> class funcref_imp;

template <typename Ret, typename... Args>
 struct funcref<Ret(Args...)>
{
    virtual Ret operator()(Args...) = 0;

    template <typename F>
     static funcref_imp<F, Ret(Args...)> make(F &&f);
};

template <typename F, typename Ret, typename... Args>
 class funcref_imp<F, Ret(Args...)> : public funcref<Ret(Args...)>
{
private:
    F f;
public:
    funcref_imp(F &&f) : f{std::move(f)} {}

    Ret operator()(Args... args) override
    {
        return f(std::forward<Args>(args)...);
    }
};

template <typename Ret, typename... Args>
 template <typename F>
auto funcref<Ret(Args...)>::make(F &&f) -> funcref_imp<F, Ret(Args...)>
{
    return {std::move(f)};
}

#endif // __cplusplus

//------------------------------------------------------------------------------
#endif // MISCUTILS_H
