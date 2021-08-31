/**
 * @file addMS.h
 * @author your name (you@domain.com)
 * @brief add mechanical system
 * @version 0.1
 * @date 2021-08-28
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

class MSFactory {
public:
    enum class MSType {
        ExampleMST
        // PendulumT,
        // BallOnParabolaNoSlidingT
    };
    static
    IMS2D*
    create(
        MSType type
    );
};

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