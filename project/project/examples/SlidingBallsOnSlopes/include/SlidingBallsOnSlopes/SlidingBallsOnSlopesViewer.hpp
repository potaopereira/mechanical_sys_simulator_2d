/**
 * @file SlidingBallsOnSlopesViewer.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Viewer for sliding balls on slopes mechanical system
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// SlidingBallsOnSlopesSolver
#include "SlidingBallsOnSlopes/SlidingBallsOnSlopesSolver.hpp"

// IMS2DImpl
#include <msinterface/interface.hpp>

// MSView<N>
#include <msviewer/viewerimpl.hpp>

/**
 * @brief A sliding-balls-on-slopes mechanical system is composed of 2 rigid bodies with 2 sliding constraints
 * 
 */
typedef MSView<2> MSVIEW2;

/**
 * @brief Class that shows visually sliding balls on slopes
 * 
 * @details A sliding-balls-on-slopes mechanical system is composed of 2 rigid bodies with 2 sliding constraints
 */
class SlidingBallsOnSlopesViewer:
public SlidingBallsOnSlopesSolver, // this is the solver
public MSVIEW2, // this is the viewer
public IMS2D // link viewer with solver
{
public:
    SlidingBallsOnSlopesViewer();
    virtual void reset();

    /**
     * @brief use this instead of IMS2DViewer::showExtraAtStep(int)
     * 
     */
    virtual void showExtraAtStep(
        int step
    );
private:
    QGraphicsLineItem* slope_1;
    QGraphicsLineItem* slope_2;
    QGraphicsLineItem* p1Top2;
};