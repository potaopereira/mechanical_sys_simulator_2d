// std::cout
#include <iostream>

// cppunit stuff
#include <cppunit/extensions/HelperMacros.h>

// CppUnit::TestSuite
#include <cppunit/TestSuite.h>

// CppUnit::TestResult
#include <cppunit/TestResult.h>

#include <cppunit/ui/text/TestRunner.h>

class ClassTest:
public CppUnit::TestFixture {

    CPPUNIT_TEST_SUITE( ClassTest );
        CPPUNIT_TEST( test );
        // CPPUNIT_TEST_EXCEPTION( someOtherMethod, exception );
    CPPUNIT_TEST_SUITE_END();
public:
    void setUp()
    {
        // initializations
        // r1 = Eigen();
    }

    void tearDown()
    {
        // destructions
    }

    void test()
    {
        std::cout << "\nTest";

        CPPUNIT_ASSERT(true);

    }


private:

};

CPPUNIT_TEST_SUITE_REGISTRATION( ClassTest );
