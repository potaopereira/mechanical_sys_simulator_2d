/**
 * @file BallOnSlopeViewer.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Viewer for a ball on a slope
 * \image html images/ball_vs_ellipse_on_slope.gif
 * \image html images/ball_vs_flatellipse_on_slope.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// BallOnSlopeSolver
#include "BallOnSlope/BallOnSlopeSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

/**
 * @brief A ball on a slope is a mechanical system composed of 1 rigid body and 1 non-sliding constraint
 * 
 */
typedef MSView<1> MSVIEW1;

/**
 * @brief Class that shows visually a ball on a slope
 * 
 * @details A ball on a slope is a mechanical system composed of 1 rigid body and 1 non-sliding constraint
 * \image html images/ball_vs_ellipse_on_slope.gif
 * \image html images/ball_vs_flatellipse_on_slope.gif
 */
class BallOnSlopeViewer:
public BallOnSlopeSolver, // this is the solver
public MSVIEW1, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    BallOnSlopeViewer();
    virtual void reset();
private:
    QGraphicsLineItem* mSlope;
};