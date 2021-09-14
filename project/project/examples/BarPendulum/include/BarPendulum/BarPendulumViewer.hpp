/**
 * @file BarPendulumViewer.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Viewer for bar pendulum mechanical system
 * @details \image html images/bar_pendulum.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// BarPendulumSolver
#include "BarPendulum/BarPendulumSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

/**
 * @brief A bar pendulum mechanical system is composed of 1 rigid body and two holonomic constraints
 * 
 */
typedef MSView<1> MSVIEW1;

/**
 * @brief Class that shows visually a bar pendulum
 * 
 * @details A bar pendulum mechanical system is composed of 1 rigid body and two holonomic constraints
 * \image html images/bar_pendulum.gif
 */
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