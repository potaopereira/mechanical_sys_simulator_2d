/**
 * @file SlidingBallOnCircleViewer.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Viewer for a sliding ball on a circle
 * @details \image html images/ellipse_on_ellipse.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// SlidingBallOnCircleSolver
#include "SlidingBallOnCircle/SlidingBallOnCircleSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

/**
 * @brief A sliding ball on a circle is a mechanical system with 1 rigid body and 1 sliding constraint
 * 
 */
typedef MSView<1> MSVIEW1;

/**
 * @brief Class that shows visually a sliding ball on a circle
 * 
 * @details A sliding ball on a circle is a mechanical system with 1 rigid body and 1 sliding constraint
 * \image html images/ellipse_on_ellipse.gif
 */
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