/**
 * @file BallsOnSlopes.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Balls on slopes mechanical system
 * @details \image html images/balls_on_slopes.gif
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
 * @brief A balls-on-slopes mechanical system is composed of 2 rigid bodies with 2 non sliding constraints
 * 
 */
typedef MRB2DSYMB<2, 1, 2, 2> BallsOnSlopesSymbImpl;

/**
 * @brief Class that defines the constraints of balls on slopes
 * 
 * @details A balls on slopes mechanical system is composed of 2 rigid bodies with 2 non sliding constraints
 * \image html images/balls_on_slopes.gif
 */
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