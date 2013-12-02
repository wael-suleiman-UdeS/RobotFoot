/**
  ******************************************************************************
  * @file    FIFOalloc.hpp
  * @author  James-Adam Renquinha Henri (Jarhmander)
  * @version 0.1
  * @date    2013-11-01
  * @brief   ---
  ******************************************************************************
  */


#ifndef FIFOALLOC_HPP
#define FIFOALLOC_HPP
//------------------------------------------------------------------------------
#include <cstdint>
#include <utility>
#include <cstring>
//------------------------------------------------------------------------------


struct anyptr
{
    anyptr(void *ptr = nullptr) : ptr{ptr} {}

    operator void *() const { return ptr; }
    template <typename T> operator T*() const { return static_cast<T*>(ptr); }
    void *ptr;
};

template <size_t> class FIFOalloc;



struct region
{
    anyptr  data;
    size_t  len;
};

struct FIFOallocator
{
    template <size_t> friend class ::FIFOalloc;
private:
    struct memblock
    {
        uint32_t   size;        // min 4
        uint32_t   top  = 0;    // Producer: rw - Consumer: ro
        uint32_t   used = 0;    // Producer: ro - Consumer: rw
        uint32_t   free = 0;    // Producer: rw - Consumer: ro
    };

    memblock mem;

    uint8_t data[sizeof(uint32_t)];     // NOT its true size!


    void displace(uint32_t blocki, size_t s, uint32_t &i);

    void adjustused();

    uint8_t *ptr_allocd()   { return data + mem.used; }
    uint8_t *ptr_unallocd() { return data + mem.free; }

    uint8_t *pushalloc(uint8_t *ptr, uint32_t sz, uint32_t newfreeblk);

    uint8_t *pushalloc(uint32_t sz, uint32_t newfreeblk)
    {
        return pushalloc(ptr_unallocd(), sz, newfreeblk);
    }

    FIFOallocator(size_t s) { mem.size = s; }
public:
    bool  isFree() const
    {
        const uint32_t free = mem.free,
                       used = mem.used,
                       top  = mem.top;

        return used == free or (!free and used and used >= top);
    }

    // Producer thread can use this method
    anyptr alloc(uint32_t sz);

    // Producer thread may use this method (sparingly)
    void dealloc(void *p);

    // Consumer thread can use this method
    void pop();
    // Consumer thread can use this method
    region access();
};


// Thin class
template <size_t S>
 class FIFOalloc : public FIFOallocator
{
    using base = FIFOallocator;
    enum { basebufsz = sizeof base::data  };

    static_assert(S >= basebufsz, "S must be >= sizeof data");
private:

    uint8_t extrabuf[S - basebufsz];
public:
    FIFOalloc() : base{S} {}
    using region = ::region;
};


class fifo_ptr
{
    FIFOallocator  *fifo;
    anyptr          ptr;
public:
    anyptr release()
    {
        auto p = ptr;
        ptr = nullptr;
        return p;
    }
    void reset(anyptr p = nullptr)
    {
        if (ptr)
        {
            fifo->dealloc(ptr);
        }
        ptr = p;
    }
    ~fifo_ptr() { reset(); }

    fifo_ptr() : fifo(nullptr), ptr(nullptr) {}
    fifo_ptr(FIFOallocator &f, size_t s)
     : fifo(&f)
    {
        ptr = fifo->alloc(s);
    }
    fifo_ptr(const fifo_ptr&) = delete;
    fifo_ptr &operator=(const fifo_ptr&) = delete;
    fifo_ptr(fifo_ptr &&o) : fifo(o.fifo), ptr(o.release()) {}
    fifo_ptr &operator=(fifo_ptr &&o)
    {
        fifo = o.fifo;
        ptr = o.release();
        return *this;
    }

    operator anyptr()       { return ptr; }
    operator anyptr() const { return ptr; }
    anyptr get()       { return ptr; }
    anyptr get() const { return ptr; }

    explicit operator bool () const { return ptr; }
    bool operator!() const { return !ptr; }
};


//------------------------------------------------------------------------------
#endif // FIFOALLOC_HPP

