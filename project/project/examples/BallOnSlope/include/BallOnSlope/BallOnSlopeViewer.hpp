#pragma once

// BallOnSlopeSolver
#include "BallOnSlope/BallOnSlopeSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

typedef MSView<1> MSVIEW1;

class BallOnSlopeViewer:
public BallOnSlopeSolver, // this is the solver
public MSVIEW1, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    BallOnSlopeViewer();
    virtual void reset();
};