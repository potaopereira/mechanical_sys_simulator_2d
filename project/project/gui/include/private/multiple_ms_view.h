/**
 * @file multiple_ms_view.h
 * @author your name (you@domain.com)
 * @brief widget to visualize several mechanical systems
 * @version 0.1
 * @date 2021-09-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// MSOptions
#include "private/mechanical_system_options.h"

// GlobalOptions
#include "private/global_options.h"

// TimerSlider
#include "private/timer_slider.h"

// SolversUpdateTimer
#include "private/solver.h"

// std::vector
#include <vector>

// std::pair
#include <utility>

// YAML::Node
#include <yaml-cpp/yaml.h>

// QWheelEvent
#include <QWheelEvent>

// QGraphicsView
#include <QGraphicsView>

class AdaptableQGraphicsView:
public QGraphicsView
{
public:
    AdaptableQGraphicsView(
        QGraphicsScene *scene,
        double scale = 1.05,
        QWidget *parent = nullptr
    );

    virtual
    void
    wheelEvent(
        QWheelEvent *ev
    );

    // virtual
    // void
    // mouseMoveEvent(
    //     QMouseEvent *ev
    // );

private:
    double mScale;
    double mScaleInv;
};

class MultipleMSView:
public QWidget
{
    Q_OBJECT

public:
    MultipleMSView(
        YAML::Node const & node,
        QWidget *parent = 0
    );
    ~MultipleMSView();

    void
    addMS(
        IMS2D*
    );

    YAML::Node getNode();

private slots:
    void changeTime(
        int value
    );

public slots:
    void solve(
        GlobalOptions::solver_options_t const & solver_options
    );

private:
    YAML::Node mNode;
    QVBoxLayout mQVBoxLayout;
    QGraphicsScene mQGraphicsScene;
    AdaptableQGraphicsView mQGraphicsView;
    int mSteps;
    TimerSlider mTimerSlider;
    GlobalOptions* mGlobalOptions;
    QScrollArea* mQScrollArea; // will own mGlobalOptions
    QTabWidget mQTabWidget; // will own mQScrollArea
    std::vector<std::tuple<IMS2D*, QScrollArea*, MSOptions*>> mList;
    SolversUpdateTimer mSolversUpdateTimer;
};
