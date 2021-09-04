#pragma once

// MRB2DSYMBHOL
#include <mssolver/rb2dSymb_specilized.hpp>

// IMRB2DPARAM
#include <msinterface/param.hpp>

typedef MRB2DSYMBHOL<1, 2> BarPendulumSymbImpl;

class BarPendulumSymb:
public BarPendulumSymbImpl,
public IMRB2DPARAM
{
public:


    BarPendulumSymb();

    /**************************************************************************/
    /* IMRB2DPARAM                                                            */
    /**************************************************************************/
    map_t& getFloatParameters() {
        return mParamList;
    };

    // do not make this private
    float p1[2];
    float p2[2];
    float length1_bar;
    float length2_bar;
    float arm_1_squared;
    float arm_2_squared;
    map_t mParamList;

};