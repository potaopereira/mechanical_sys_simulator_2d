/**
 * @file PointOnSuperEllipseViewer.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Viewer for point on super-ellipse mechanical system
 * @details \image html images/point_on_superellipse.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// PointOnSuperEllipseSolver
#include "PointOnSuperEllipse/PointOnSuperEllipseSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

/**
 * @brief A point on super-ellipse mechanical system is composed of 1 rigid body and 1 non-sliding constraint
 * 
 */
typedef MSView<1> MSVIEW1;

/**
 * @brief Class that shows visually a point on a super-ellipse
 * 
 * @details A point on super-ellipse mechanical system is composed of 1 rigid body and 1 non-sliding constraint
 * \image html images/point_on_superellipse.gif
 */
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