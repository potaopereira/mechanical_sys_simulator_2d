/**
 * @file BallOnCircle.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Non sliding ball on a circle
 * @details \image html images/ellipse_on_ellipse.gif
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
 * @brief A non sliding ball on a circle is a mechanical system with 1 rigid body and 1 non-sliding constraint
 * 
 */
typedef MRB2DSYMB<1, 0, 1, 1> BallOnCircleSymbImpl;

/**
 * @brief Class that defines the constraints of a non sliding ball on a circle
 * 
 * @details A non sliding ball on a circle is a mechanical system with 1 rigid body and 1 non-sliding constraint
 * \image html images/ellipse_on_ellipse.gif
 */
class BallOnCircleSymb:
public BallOnCircleSymbImpl,
public IMRB2DPARAM
{
public:


    BallOnCircleSymb();

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