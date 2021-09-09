#pragma once

// SlidingBallOnSlopeSolver
#include "SlidingBallOnSlope/SlidingBallOnSlopeSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

typedef MSView<1> MSVIEW1;

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