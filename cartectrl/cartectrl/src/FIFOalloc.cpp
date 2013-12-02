/**
  ******************************************************************************
  * @file    FIFOalloc.cpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-01
  * @brief   ---
  ******************************************************************************
  */

//------------------------------------------------------------------------------
#include "FIFOalloc.hpp"
//------------------------------------------------------------------------------


using std::uint8_t;
using std::uint32_t;
using std::ptrdiff_t;
using std::memmove;

//------------------------------------------------------------------------------
static uint8_t *markregion(uint8_t *ptr, size_t sz)
{
    reinterpret_cast<uint32_t &>(*ptr) = sz;
    return ptr + sizeof(uint32_t);
}
//------------------------------------------------------------------------------
static region getregion(uint8_t *ptr)
{
    const auto sz = reinterpret_cast<const uint32_t &>(*ptr);
    return { ptr + sizeof sz, sz };
}
//------------------------------------------------------------------------------
static uint32_t adjustsize(uint32_t sz)
{
    enum { tagsize = sizeof sz };
    // Trick to make size aligned to (32 bits) boundaries, try for yourself
    // for example: (sz -> ret)
    // 1 -> 4, ..., 4 -> 4, 5 -> 8, ...
    return (sz + (tagsize - 1)) & ~(tagsize - 1);
}
//------------------------------------------------------------------------------
static uint32_t sizewithtag(uint32_t sz)
{
    return adjustsize(sz + sizeof sz);
}
//------------------------------------------------------------------------------
void FIFOallocator::displace(uint32_t blocki, size_t s, uint32_t &i)
{
    const auto diff = i - (blocki + s);
    memmove(data+blocki, data+blocki+s, diff);
    i -= s;
}
//------------------------------------------------------------------------------
void FIFOallocator::adjustused()
{
    if (mem.used > mem.free and mem.used >= mem.top)
        mem.used = 0;
}
//------------------------------------------------------------------------------
uint8_t *FIFOallocator::pushalloc(uint8_t *ptr, uint32_t sz, uint32_t newfreeblk)
{
    mem.free = newfreeblk;
    return markregion(ptr, sz);
}
//------------------------------------------------------------------------------
anyptr FIFOallocator::alloc(uint32_t sz)
{
    // Is O(1)
    const uint32_t memused     = mem.used;
    const ptrdiff_t adjustedsz = sizewithtag(sz);
    const bool warp_possible   = memused <= mem.free;

    const ptrdiff_t prevused   = (ptrdiff_t)memused - 1;
    const ptrdiff_t upperbound = warp_possible ?
                                    mem.size
                                  : prevused;

    const ptrdiff_t fwdspace = upperbound - mem.free;

    if (fwdspace >= adjustedsz)
    {
        return pushalloc(sz, mem.free + adjustedsz);
    }
    // implicit else
    const auto frontspace = prevused;

    if (warp_possible and (frontspace >= adjustedsz))
    {
        mem.top = mem.free;
        return pushalloc(data, sz, adjustedsz);
    }

    // Could not allocate.
    return nullptr;
}
//------------------------------------------------------------------------------
void FIFOallocator::dealloc(void *p)
{
    // Warning: this code is O(n), and assume p is allocated from the
    // buffer, plus it may invalidate other pointers into the buffer --- use
    // with caution.
    if (p == nullptr) return;

    const uint32_t memused  = mem.used;
    const auto ptr          = static_cast<uint8_t *>(p) - sizeof(uint32_t);
    const auto sz           = reinterpret_cast<const uint32_t &>(*ptr);
    const auto adjustedsz   = sizewithtag(sz);

    const auto blocki = ptr - data;

    if (mem.free > memused or mem.top <= memused)
    {
        // normal, unwrapped case
        displace(blocki, adjustedsz, mem.free);
    }
    else
    {
        // wrapped case, we need more info in order to do things correctly
        if (blocki < (ptrdiff_t)mem.free)
        {
            displace(blocki, adjustedsz, mem.free);
        }
        else
        {
            // ahead of mem.used, so we just manipulate mem.top
            displace(blocki, adjustedsz, mem.top);
        }
    }

}
//------------------------------------------------------------------------------
void FIFOallocator::pop()
{
    // O(1)
    const auto reg        = access();
    const uint32_t sz     = reg.len;
    const auto adjustedsz = sizewithtag(sz);

    mem.used = mem.used + adjustedsz;
}
//------------------------------------------------------------------------------
region FIFOallocator::access()
{
    // O(1)
    adjustused();
    return getregion(ptr_allocd());
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
