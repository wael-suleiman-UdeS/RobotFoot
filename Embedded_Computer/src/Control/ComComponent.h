#ifndef COM_COMPONENT_H
#define COM_COMPONENT_H

#include <cstdint>
#include <vector>
#include <assert.h>

class ComComponent
{
public:
   ComComponent(){};
   virtual ~ComComponent(){};

   virtual void Read(const std::vector<std::uint8_t>& msg)
   {
      // Should be implemented be child class if used.
      assert(0);
   }
   virtual void Write(std::vector<std::uint8_t>& msg) = 0;
};

#endif //COM_COMPONENT
