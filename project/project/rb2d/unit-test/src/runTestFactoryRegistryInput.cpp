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

// M_PI
#include <cmath>

class RB2DTest:
public CppUnit::TestFixture {

    CPPUNIT_TEST_SUITE( RB2DTest );
        CPPUNIT_TEST( getLinearPosition );
        CPPUNIT_TEST( getAngularPosition );
        // CPPUNIT_TEST_EXCEPTION( someOtherMethod, exception );
    CPPUNIT_TEST_SUITE_END();
public:
    void setUp()
    {
        // initializations
        // r1 = RB2D();
    }

    void tearDown()
    {
        // destructions
    }

    void getLinearPosition()
    {
        std::cout << "\nTest get linear position";
        LinearPosition p = r1.getLP();
        CPPUNIT_ASSERT(p.get0()==0);
        CPPUNIT_ASSERT(p.get1()==0);
    }

    void getAngularPosition()
    {
        std::cout << "\nTest get linear position";
        AngularPosition p = r1.getAP();
        CPPUNIT_ASSERT(p.get00()==1);
        CPPUNIT_ASSERT(p.get01()==0);
        CPPUNIT_ASSERT(p.get10()==0);
        CPPUNIT_ASSERT(p.get11()==1);

    }

    void getAngularPosition90Degress()
    {
        std::cout << "\nTest get linear position at 90 degrees";
        double pi_2 = M_PI/2;
        AngularPosition p(pi_2);
        CPPUNIT_ASSERT(p.get00()==0);
        CPPUNIT_ASSERT(p.get01()==-1);
        CPPUNIT_ASSERT(p.get10()==+1);
        CPPUNIT_ASSERT(p.get11()==0);

    }

    void someOtherMethod()
    {
        std::cout << "\nTest some other method that throws exception";

    }

private:
    // some private members
    RB2D r1;
};

CPPUNIT_TEST_SUITE_REGISTRATION( RB2DTest );
