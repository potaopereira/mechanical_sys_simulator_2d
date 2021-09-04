#pragma once

// SlidingBallOnCircleSolver
#include "SlidingBallOnCircle/SlidingBallOnCircleSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

typedef MSView<1> MSVIEW1;

class SlidingBallOnCircleViewer:
public SlidingBallOnCircleSolver, // this is the solver
public MSVIEW1, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    SlidingBallOnCircleViewer();
    virtual void reset();
private:
    std::vector<QGraphicsItem*> mListOfConstraints;
    QGraphicsEllipseItem* mEllipse;
};