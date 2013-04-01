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

    UnitTest( char* testName )
    {
        TestName = testName;
    }


    virtual ~UnitTest(){};

    void Run()
    {
        UnitTestOutput::SendMessage( "\r\nBegining " );
        UnitTestOutput::SendMessage( TestName );
        UnitTestOutput::SendMessage( " Unit Test." );

        if ( Test() )
        {
            UnitTestOutput::SendMessage( "\r\n" );
            UnitTestOutput::SendMessage( TestName );
            UnitTestOutput::SendMessage( " : " );
            UnitTestOutput::SendMessage( UnitTestString::UNIT_TEST_PASSED );
        }
        else
        {
            UnitTestOutput::SendMessage( "\r\n" );
            UnitTestOutput::SendMessage( TestName );
            UnitTestOutput::SendMessage( " : " );
            UnitTestOutput::SendMessage( UnitTestString::UNIT_TEST_FAILED );
        }
    }

    virtual bool Test() = 0;

private:

    char* TestName;

};


class Test
{
public:
    Test(){}
    ~Test(){}
};

#endif // UNIT_TEST_H
