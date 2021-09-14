/**
 * @file BarPendulum.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Bar pendulum mechanical system
 * @details \image html images/bar_pendulum.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// MRB2DSYMBHOL
#include <mssolver/rb2dSymb_specilized.hpp>

// IMRB2DPARAM
#include <msinterface/param.hpp>

/**
 * @brief A bar pendulum mechanical system is composed of 1 rigid body and two holonomic constraints
 * 
 */
typedef MRB2DSYMBHOL<1, 2> BarPendulumSymbImpl;

/**
 * @brief Class that defines the constraints of a bar pendulum
 * 
 * @details A bar pendulum mechanical system is composed of 1 rigid body and two holonomic constraints
 * \image html images/bar_pendulum.gif
 */
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