/**
 * @file SlidingBallOnSlopeViewer.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Viewer for a sliding ball on a slope
 * @details \image html images/ball_vs_ellipse_on_slope.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// SlidingBallOnSlopeSolver
#include "SlidingBallOnSlope/SlidingBallOnSlopeSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

/**
 * @brief A sliding ball on a slope is a mechanical system composed of 1 rigid body and 1 sliding constraint
 * 
 */
typedef MSView<1> MSVIEW1;

/**
 * @brief Class that shows visually a sliding ball on a slope
 * 
 * @details A sliding ball on a slope is a mechanical system composed of 1 rigid body and 1 sliding constraint
 * \image html images/ball_vs_ellipse_on_slope.gif
 */
class SlidingBallOnSlopeViewer:
public SlidingBallOnSlopeSolver, // this is the solver
public MSVIEW1, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    SlidingBallOnSlopeViewer();
    virtual void reset();
private:
    QGraphicsLineItem* mSlope;
};