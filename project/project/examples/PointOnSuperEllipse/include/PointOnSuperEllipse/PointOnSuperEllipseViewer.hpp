#pragma once

// PointOnSuperEllipseSolver
#include "PointOnSuperEllipse/PointOnSuperEllipseSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

typedef MSView<1> MSVIEW1;

class PointOnSuperEllipseViewer:
public PointOnSuperEllipseSolver, // this is the solver
public MSVIEW1, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    PointOnSuperEllipseViewer();
    virtual void reset();
private:
    std::vector<QGraphicsItem*> mListOfConstraints;
    QGraphicsPathItem* mSuperEllipse;
};