#include "Control/Protocol.h"

#include <iostream>
#include <vector>
#include <iterator>

int main()
{
    std::vector<char> test;

    test.push_back(0xff);
    test.push_back(0x2f);
    
    test.push_back(0x01);
    test.push_back(0x01);
    test.push_back(0x21);
    test.push_back(0x01);

    int size = Protocol::CalculCheckSum(test);

    std::cout << size << std::endl;

    copy(test.begin(), test.end(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;
   
    std::vector<char> result;
    Protocol::GenerateDataMsg(Protocol::ButtonHeader,test,result);
    copy(result.begin(), result.end(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;
}
