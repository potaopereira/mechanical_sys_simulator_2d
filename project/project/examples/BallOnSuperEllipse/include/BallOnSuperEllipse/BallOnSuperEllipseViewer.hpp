#pragma once

// BallOnSuperEllipseSolver
#include "BallOnSuperEllipse/BallOnSuperEllipseSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

typedef MSView<1> MSVIEW1;

class BallOnSuperEllipseViewer:
public BallOnSuperEllipseSolver, // this is the solver
public MSVIEW1, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    BallOnSuperEllipseViewer();
    virtual void reset();
private:
    std::vector<QGraphicsItem*> mListOfConstraints;
    QGraphicsPathItem* mSuperEllipse;
};