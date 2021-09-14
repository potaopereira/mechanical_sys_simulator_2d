/**
 * @file SlidingBallOnSlope.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Sliding ball on a slope
 * @details \image html images/ball_vs_ellipse_on_slope.gif
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
 * @brief A sliding ball on a slope is a mechanical system composed of 1 rigid body and 1 sliding constraint
 * 
 */
typedef MRB2DSYMB<1, 0, 1, 0> SlidingBallOnSlopeSymbImpl;

/**
 * @brief Class that defines the constraints of a sliding ball on a circle
 * 
 * @details A sliding ball on a slope is a mechanical system composed of 1 rigid body and 1 sliding constraint
 * \image html images/ball_vs_ellipse_on_slope.gif
 */
class SlidingBallOnSlopeSymb:
public SlidingBallOnSlopeSymbImpl,
public IMRB2DPARAM
{
public:


    SlidingBallOnSlopeSymb();

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