// complex
#include <rb2d/rb2d.hpp>

// std::cout
#include <iostream>

// cppunit stuff
#include <cppunit/extensions/HelperMacros.h>

// CppUnit::TestSuite
#include <cppunit/TestSuite.h>

// CppUnit::TestResult
#include <cppunit/TestResult.h>

#include <cppunit/ui/text/TestRunner.h>


class RB2DTest:
public CppUnit::TestFixture {

    CPPUNIT_TEST_SUITE( RB2DTest );
        CPPUNIT_TEST( someMethod );
        // CPPUNIT_TEST_EXCEPTION( someOtherMethod, exception );
    CPPUNIT_TEST_SUITE_END();
public:
    void setUp()
    {
        // initializations
    }

    void tearDown()
    {
        // destructions
    }

    void someMethod()
    {
        std::cout << "\nTest some Method";
        // CPPUNIT_ASSERT( );
    }

    void someOtherMethod()
    {
        std::cout << "\nTest some other method that throws exception";

    }

private:
    // some private members
};

CPPUNIT_TEST_SUITE_REGISTRATION( RB2DTest );
