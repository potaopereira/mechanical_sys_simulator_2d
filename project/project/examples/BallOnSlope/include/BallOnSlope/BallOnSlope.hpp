/**
 * @file BallOnSlope.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Ball on a slope
 * @details 
 * \image html images/ball_vs_ellipse_on_slope.gif
 * \image html images/ball_vs_flatellipse_on_slope.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <mssolver/rb2dSymb.hpp>

// IMRB2DPARAM
#include <msinterface/param.hpp>

/**
 * @brief A ball on a slope is a mechanical system composed of 1 rigid body and 1 non-sliding constraint
 * 
 */
typedef MRB2DSYMB<1, 0, 1, 1> BallOnSlopeSymbImpl;

/**
 * @brief Class that defines the constraints of a ball on a slope
 * 
 * @details A ball on a slope is a mechanical system composed of 1 rigid body and 1 non-sliding constraint
 * \image html images/ball_vs_ellipse_on_slope.gif
 * \image html images/ball_vs_flatellipse_on_slope.gif
 */
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
    float offset1;
    float offset2;
    float radius1;
    float radius2;
    map_t mParamList;

};