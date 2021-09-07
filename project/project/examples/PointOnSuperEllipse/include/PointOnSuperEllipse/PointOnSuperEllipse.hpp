#pragma once

// MRB2DSYMBHOL
#include <mssolver/rb2dSymb_specilized.hpp>

// IMRB2DPARAM
#include <msinterface/param.hpp>

typedef MRB2DSYMBHOL<1, 1> PointOnSuperEllipseSymbImpl;

class PointOnSuperEllipseSymb:
public PointOnSuperEllipseSymbImpl,
public IMRB2DPARAM
{
public:


    PointOnSuperEllipseSymb();

    /**************************************************************************/
    /* IMRB2DPARAM                                                            */
    /**************************************************************************/
    map_t& getFloatParameters() {
        return mParamList;
    };

    // do not make this private
    float ball_r;
    float outside_rinv;
    float outside_alpha;

    map_t mParamList;

};