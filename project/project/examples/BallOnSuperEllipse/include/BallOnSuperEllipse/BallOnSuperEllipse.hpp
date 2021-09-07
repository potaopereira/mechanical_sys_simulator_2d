#pragma once

// MRB2DSYMB
#include <mssolver/rb2dSymb.hpp>

// IMRB2DPARAM
#include <msinterface/param.hpp>

typedef MRB2DSYMB<1, 0, 1, 1> BallOnSuperEllipseSymbImpl;

class BallOnSuperEllipseSymb:
public BallOnSuperEllipseSymbImpl,
public IMRB2DPARAM
{
public:


    BallOnSuperEllipseSymb();

    /**************************************************************************/
    /* IMRB2DPARAM                                                            */
    /**************************************************************************/
    map_t& getFloatParameters() {
        return mParamList;
    };

    // do not make this private
    float ball_r;
    float outside_a;
    float outside_alpha;

    map_t mParamList;

};