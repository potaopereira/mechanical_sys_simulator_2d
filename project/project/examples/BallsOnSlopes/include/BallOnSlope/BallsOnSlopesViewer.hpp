#pragma once

// BallsOnSlopesSolver
#include "BallsOnSlopes/BallsOnSlopesSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

typedef MSView<1> MSVIEW1;

class BallsOnSlopesViewer:
public BallsOnSlopesSolver, // this is the solver
public MSVIEW1, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    BallsOnSlopesViewer();
    virtual void reset();
private:
    QGraphicsLineItem* slope_1;
    QGraphicsLineItem* slope_2;
};