#include "Control/Protocol.h"

#include <iostream>
#include <vector>
#include <iterator>
#include <string.h>

void DisplayVector(const std::vector<char>& vect)
{
    copy(vect.begin(), vect.end(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;
}

void TestProtocol()
{
    std::vector<char> test;

    test.push_back(0x01);
    test.push_back(0x01);
    test.push_back(0x21);
    test.push_back(0x01);

    std::cout << "First msg" << std::endl;
    DisplayVector(test);

    std::vector<char> msg1;
    Protocol::GenerateDataMsg(Protocol::MotorHeader,test,msg1);
    
    std::cout << "Complete First Msg" << std::endl;
    DisplayVector(msg1);
   
    test.clear();
    test.push_back(0x02); 
    test.push_back(0x02); 
    test.push_back(0x02); 
    test.push_back(0x02); 
    test.push_back(0x02); 
    
    std::cout << "Second msg" << std::endl;
    DisplayVector(test);
    
    std::vector<char> msg2;
    Protocol::GenerateDataMsg(Protocol::MsgHeader,test,msg2);
    
    std::cout << "Complete Second Msg" << std::endl;
    DisplayVector(msg2);

    /*int size = Protocol::CalculCheckSum(msg2);
    std::cout << "Checksum" << std::endl;
    std::cout << size << std::endl;*/

    msg1.insert(msg1.end(), msg2.begin(), msg2.end());

    std::cout << "Full Data Msg" << std::endl;
    DisplayVector(msg1);
    
    std::vector<char> packet;
    std::vector<char>::const_iterator itr = msg1.begin();
    const std::vector<char>::const_iterator end = msg1.end(); 
    
    Protocol::ReadPacket(Protocol::MsgHeader, itr, end, packet);

    std::cout << "Packet1" << std::endl;
    DisplayVector(packet);

    packet.clear();
    Protocol::ReadPacket(Protocol::MsgHeader, itr, end, packet);

    std::cout << "Packet2" << std::endl;
    DisplayVector(packet);


}

struct Test
{
    int m1;
    int m2;
    int m3;
};

void TestMemCopy()
{
   Test test = Test{0,0,0};
   //int ArrayTest[] = {1,2,3};
   std::vector<int> vect;
   vect.push_back(4);
   vect.push_back(5);
   vect.push_back(6);
   std::cout << test.m1 << test.m2 << test.m3 << std::endl;
  
  void* ptr = &test;
  //void* ptr2 = vect.begin();
 //   *ptr = *(vect.begin()); 
    memcpy(ptr,vect.data(),sizeof(vect));
   //std::copy(vect.begin(),vect.end(),ArrayTest);
    //std::copy(vect.begin(),vect.end(),ptr);
   //memcpy(&test,ArrayTest,sizeof(ArrayTest));
   std::cout << test.m1 << test.m2 << test.m3;
}

int main()
{
    TestMemCopy();
}
