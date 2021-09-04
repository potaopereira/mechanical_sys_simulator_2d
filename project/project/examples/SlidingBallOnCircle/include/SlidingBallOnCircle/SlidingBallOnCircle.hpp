#pragma once

#include <mssolver/rb2dSymb.hpp>

// IMRB2DPARAM
#include <msinterface/param.hpp>

typedef MRB2DSYMB<1, 0, 1, 0> SlidingBallOnCircleSymbImpl;

class SlidingBallOnCircleSymb:
public SlidingBallOnCircleSymbImpl,
public IMRB2DPARAM
{
public:


    SlidingBallOnCircleSymb();

    /**************************************************************************/
    /* IMRB2DPARAM                                                            */
    /**************************************************************************/
    map_t& getFloatParameters() {
        return mParamList;
    };

    // do not make this private
    float ball_a0;
    float ball_a1;

    float ball_offset0;
    float ball_offset1;

    float outside_a0;
    float outside_a1;

    map_t mParamList;

};