/**
 * @file PointOnSuperEllipse.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Point on super-ellipse mechanical system
 * @details \image html images/point_on_superellipse.gif
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
 * @brief A point on super-ellipse mechanical system is composed of 1 rigid body and 1 non-sliding constraint
 * 
 */
typedef MRB2DSYMBHOL<1, 1> PointOnSuperEllipseSymbImpl;

/**
 * @brief Class that defines the constraints of a point on super-ellipse
 * 
 * @details A point on super-ellipse mechanical system is composed of 1 rigid body and 1 non-sliding constraint
 * \image html images/point_on_superellipse.gif
 */
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