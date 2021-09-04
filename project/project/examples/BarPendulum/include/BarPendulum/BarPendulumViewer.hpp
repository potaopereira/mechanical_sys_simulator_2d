#pragma once

// BarPendulumSolver
#include "BarPendulum/BarPendulumSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

typedef MSView<1> MSVIEW1;

class BarPendulumViewer:
public BarPendulumSolver, // this is the solver
public MSVIEW1, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    BarPendulumViewer();
    virtual void reset();

    /**
     * @brief use this instead of IMS2DViewer::showExtraAtStep(int)
     * 
     */
    virtual void showExtraAtStep(
        int step
    );

private:
    QGraphicsLineItem* point2contact1;
    QGraphicsLineItem* point2contact2;
};