/**
 * @file solver.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief The solver for each mechanical system runs in a separate thread
 * @details When solving several mechanical systems, they are all started simultaneously, and a single timer provides the minimum progress of all the solvers
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

// GlobalOptions
#include "private/global_options.h"

// IMS2D
#include <msinterface/interface.hpp>

// QThread
#include <QThread>

// QTimer
#include <QTimer>

// std::vector
#include <vector>

/**
 * @brief Thread to solve the ODE of a mechanical system
 * 
 */
class SolverThread:
public QThread
{
    Q_OBJECT

protected:
    void run() override;

public:
    SolverThread(
        IMS2D* MS2D,
        double time_initial,
        double time_step,
        int steps,
        int unsigned timersleep_ms = 500
    );

    int getProgrees() const;

public slots:
    void emitProgress();

signals:
    void resultReady(const QString &result);
    void setProgress(int value);

private:
    IMS2D* mMS2D;
    double mTimeInitial;
    double mTimeStep;
    int mSteps;
    int mCurrentStep;
    QTimer mQTimer;
};

/**
 * @brief Timer used to check the progress of the solving of the ODEs of all the mechanical systems
 * 
 */
class SolversUpdateTimer:
public QTimer
{
    Q_OBJECT
public:
    SolversUpdateTimer(
        int unsigned timersleep_ms = 500,
        QObject *parent = nullptr
    );

    bool isFinished() const { return mFinished; }

    bool startSolvers(
        std::vector<IMS2D*> list,
        GlobalOptions::solver_options_t solver_options
    );

public slots:
    void solverupdate();

signals:
    void setProgress(int value);

private:
    int unsigned mTimerMs;
    std::vector<SolverThread*> mThreadsList;
    bool mFinished;
    int mFinalProgess;
};