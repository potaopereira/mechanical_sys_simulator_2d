#pragma once

// BallOnCircleSolver
#include "BallOnCircle/BallOnCircleSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

typedef MSView<1> MSVIEW1;

class BallOnCircleViewer:
public BallOnCircleSolver, // this is the solver
public MSVIEW1, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    BallOnCircleViewer();
    virtual void reset();
private:
    std::vector<QGraphicsItem*> mListOfConstraints;
    QGraphicsEllipseItem* mEllipse;
};