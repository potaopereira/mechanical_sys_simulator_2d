/**
 * @file BallsOnSlopesViewer.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Viewer for balls on slopes mechanical system
 * @details \image html images/balls_on_slopes.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// BallsOnSlopesSolver
#include "BallsOnSlopes/BallsOnSlopesSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

/**
 * @brief A balls-on-slopes mechanical system is composed of 2 rigid bodies with 2 non sliding constraints
 * 
 */
typedef MSView<2> MSVIEW2;

/**
 * @brief Class that shows visually balls on slopes
 * 
 * @details A balls-on-slopes mechanical system is composed of 2 rigid bodies with 2 non sliding constraints
 * \image html images/balls_on_slopes.gif
 */
class BallsOnSlopesViewer:
public BallsOnSlopesSolver, // this is the solver
public MSVIEW2, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    BallsOnSlopesViewer();
    virtual void reset();

    /**
     * @brief use this instead of IMS2DViewer::showExtraAtStep(int)
     * 
     */
    virtual void showExtraAtStep(
        int step
    );
private:
    QGraphicsLineItem* slope_1;
    QGraphicsLineItem* slope_2;
    QGraphicsLineItem* p1Top2;
};