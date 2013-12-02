#include <iostream>
#include "FIFOalloc.hpp"
#include <memory>
#include <new>
#include <cstring>
#include <stdexcept>
#include <string>
#include <sstream>
#include <iterator>
#include <type_traits>
#include <algorithm>
#include <random>

#ifdef assert
    #undef assert
#endif

#define STRINGIFY(x)    #x


#define assert(cond, ...)   \
    if (!(cond)) { except(__FILE__, __PRETTY_FUNCTION__, \
                          __LINE__, STRINGIFY(cond), ##__VA_ARGS__); }
using namespace std;

//------------------------------------------------------------------------------
namespace am
{

string v2s(string s) { return move(s); }

template <typename T>
 typename enable_if<is_arithmetic<T>::value, string>::
  type v2s(T t)
{
    return to_string(t);
}

string parseerr() { return {};}

template <typename T, typename ...Args>
 string parseerr(T&& t, Args... args)
{
    return v2s(move(t)) + parseerr(forward<Args>(args)...);
}

} // namespace am
//------------------------------------------------------------------------------
template <typename ...Args >
 void except(string file, string func, int line, string exp, Args ...args)
{
    string msg = move(file) + ":" + to_string(line) + ": "
               + move(func) + ": Error: ";

    if (sizeof...(args) == 0)
    {
        msg += "Assertion '" + exp + "' failed.";
    }
    else
    {
        msg += am::parseerr(args...);
    }

    throw runtime_error {msg};
}
//------------------------------------------------------------------------------

static random_device rd;
static mt19937       rdgen {rd()};

using uid = uniform_int_distribution<>;

//------------------------------------------------------------------------------
char random_hex_digit()
{
    static uid d {0,15};
    return "0123456789ABCDEF"[d(rdgen)];
}
//------------------------------------------------------------------------------
string random_string(int len)
{
    assert(len >= 1, "len of string is insuffisent (", len, ").");

    string s(len, 0);

    generate_n(begin(s), len, random_hex_digit);
    return s;
}
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
void test1()
{
    enum { extrasp = 32 };

    auto p = unique_ptr<FIFOalloc<extrasp>> {new FIFOalloc<extrasp> {}};
    char *seg;

    cout << "1st test..." << endl;

    // Should fail
    seg = p->alloc(32);
    assert(seg == nullptr);
    // Should succeed
    seg = p->alloc(12);
    assert(seg != nullptr);

    // Put some text in acquired buffers...
    strncpy(seg, "SOME TEXT...", 12);
    seg = p->alloc(12);
    assert(seg != nullptr);
    strncpy(seg, "OTHER TXT...", 12);
    // Verify that data matches what is pushed in.
    auto reg1 = p->access();
    assert(reg1.len == 12);
    assert(reg1.data != nullptr);
    seg = reg1.data;
    assert(memcmp(seg, "SOME TEXT...", 12) == 0);
    p->pop();
    auto reg2 = p->access();
    assert(reg2.len == 12);
    assert(reg2.data != nullptr);
    seg = reg2.data;
    assert(memcmp(seg, "OTHER TXT...", 12) == 0);
    p->pop();

    // Should be free
    assert(p->isFree());

    seg= p->alloc(12);
    assert(seg != nullptr);
    auto seg2 = p->alloc(8);
    assert(seg2 != nullptr);

    strncpy(seg,  "INVISIBLE...", 12);
    strncpy(seg2, "VISIBLE.", 8);

    p->dealloc(seg);

    reg1 = p->access();
    seg  = reg1.data;
    // We should see the 2nd message
    assert(memcmp(seg, "VISIBLE.", 8) == 0);
    p->pop();
    // Should be free again
    assert(p->isFree());
}
//------------------------------------------------------------------------------
void test2()
{
    struct nl { void operator()(void *) { cout << endl; } };

    enum { repeat = 200000 };

    unique_ptr<void,nl> _ {&cout};

    auto randomgenerator = [](int lower, int upper)
    {
        uid d {lower, upper};
        return [d]() mutable { return d(rdgen); };
    };

    auto lenrandgen = randomgenerator(1,28);
    FIFOalloc<64> buf;

    cout << "2nd test..." << endl;
    cout << ">test alloc/access/pop...";

    for (int i=0; i < repeat; ++i)
    {
        int len = lenrandgen();
        char *alloc = buf.alloc(len);
        assert(alloc, "Allocation failed.");
        auto s = random_string(len);

        memcpy(alloc, s.c_str(), len);

        auto seg   = buf.access();
        char *data = seg.data;
        assert(data == alloc);
        string result {data, data+len};
        assert(result== s, "String corrupted (", result, ")");
        buf.pop();
        assert(buf.isFree());
    }
    cout << "passed." << endl;
    //--------------------------------------------------------------------------
    cout << ">test alloc/dealloc...";

    for (int i=0; i < repeat; ++i)
    {
        int len = lenrandgen();
        auto p  = buf.alloc(len);
        assert(p, "Allocation failed.");
        buf.dealloc(p);
        assert(buf.isFree());
    }
    cout << "passed." << endl;
}
//------------------------------------------------------------------------------
int main()
try
{
    test1();
    test2();
    cout << "All tests passed." << endl;
}
catch (const exception &e)
{
    cout << e.what() << endl;
    return 1;
}
