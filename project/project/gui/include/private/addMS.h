/**
 * @file addMS.h
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Menu to add a mechanical system
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <QMenu>
#include <QAction>
#include <vector>

// IMS2D
#include <msinterface/interface.hpp>

/**
 * @brief Factory of mechanical systems
 * 
 */
class MSFactory {
public:
    enum class MSType {
        BallOnSlopeT,
        SlidingBallOnSlopeT,
        BallOnCircleT,
        SlidingBallOnCircleT,
        BarPendulumT,
        BallsOnSlopesT,
        SlidingBallsOnSlopesT,
        PointOnSuperEllipseT
        // ExampleMST,
        // PendulumT,
        // BallOnParabolaNoSlidingT
    };
    static
    IMS2D*
    create(
        MSType type
    );
};

/**
 * @brief Menu on gui where user selects which mechanical system he wants to add to the simulation
 * 
 */
class AddMS:
public QMenu
{
    Q_OBJECT
public:

    /**
     * @brief List of mechanical systems available
     * 
     */
    static
    std::vector<std::pair<std::string, MSFactory::MSType>> mList;

    AddMS(
        QWidget *parent = nullptr
    );
    virtual ~AddMS();
public slots:
    void add(
        QAction *action
    );
signals:
    void addedMS(
        IMS2D* newMS
    );
private:
    QMenu addMS;
    QActionGroup mQActionGroup;
    std::vector<QAction*> mActions;
    std::map<QAction*, MSFactory::MSType> mMap;
};