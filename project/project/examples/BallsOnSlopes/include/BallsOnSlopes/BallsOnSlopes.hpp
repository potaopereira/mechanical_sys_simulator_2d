#pragma once

#include <mssolver/rb2dSymb.hpp>

// IMRB2DPARAM
#include <msinterface/param.hpp>

typedef MRB2DSYMB<2, 1, 2, 2> BallsOnSlopesSymbImpl;

class BallsOnSlopesSymb:
public BallsOnSlopesSymbImpl,
public IMRB2DPARAM
{
public:


    BallsOnSlopesSymb();

    /**************************************************************************/
    /* IMRB2DPARAM                                                            */
    /**************************************************************************/
    map_t& getFloatParameters() {
        return mParamList;
    };

    // do not make this private
    float l_squared;
    float slope_1;
    float radius1_1;
    float radius1_2;
    float slope_2;
    float radius2_1;
    float radius2_2;
    map_t mParamList;

};