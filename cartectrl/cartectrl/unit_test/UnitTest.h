#ifndef UNIT_TEST_H
#define UNIT_TEST_H

#include "UnitTestOutput.h"

#include <string>

namespace UnitTestString
{
    const char UNIT_TEST_PASSED[] = "PASSED";
    const char UNIT_TEST_FAILED[] = "FAILED";
}

class UnitTest
{

public:
    UnitTest(){}
    UnitTest( const std::string testName )
    {
        TestName = testName;
    }

    virtual ~UnitTest(){};

    void Run()
    {
        UnitTestOutput::SendMessage( "\n\nBegining " );
        //UnitTestOutput::SendMessage( TestName.c_str() );
        UnitTestOutput::SendMessage( " Unit Test. \n" );

        if ( Test() )
        {
            //UnitTestOutput::SendMessage( TestName.c_str() );
            UnitTestOutput::SendMessage( " = " );
            //UnitTestOutput::SendMessage( UnitTestString::UNIT_TEST_PASSED );
        }
        else
        {
            //UnitTestOutput::SendMessage( TestName.c_str() );
            UnitTestOutput::SendMessage( " = " );
            //UnitTestOutput::SendMessage( UnitTestString::UNIT_TEST_FAILED );
        }
    }

    virtual bool Test() = 0;

private:

    std::string TestName;

};


class Test
{
public:
    Test(){}
    ~Test(){}
};

#endif // UNIT_TEST_H
