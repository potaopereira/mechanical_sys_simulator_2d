#pragma once

// SlidingBallsOnSlopesSolver
#include "SlidingBallsOnSlopes/SlidingBallsOnSlopesSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

typedef MSView<2> MSVIEW2;

class SlidingBallsOnSlopesViewer:
public SlidingBallsOnSlopesSolver, // this is the solver
public MSVIEW2, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    SlidingBallsOnSlopesViewer();
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