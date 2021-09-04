#pragma once

#include <mssolver/rb2dSymb.hpp>

// IMRB2DPARAM
#include <msinterface/param.hpp>

typedef MRB2DSYMB<1, 0, 1, 1> BallOnSlopeSymbImpl;

class BallOnSlopeSymb:
public BallOnSlopeSymbImpl,
public IMRB2DPARAM
{
public:


    BallOnSlopeSymb();

    /**************************************************************************/
    /* IMRB2DPARAM                                                            */
    /**************************************************************************/
    map_t& getFloatParameters() {
        return mParamList;
    };

    // do not make this private
    float slope;
    float radius1;
    float radius2;
    map_t mParamList;

};