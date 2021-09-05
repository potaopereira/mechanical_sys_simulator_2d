#include "private/solver.h"

// min
#include <algorithm>

// std::cout, std::endl
#include <iostream>

SolverThread::SolverThread(
    IMS2D* MS2D,
    double time_initial,
    double time_step,
    int steps,
    int unsigned timersleep_ms
):
mMS2D(MS2D),
mTimeInitial(time_initial),
mTimeStep(time_step),
mSteps(steps),
mCurrentStep(0),
mQTimer()
{
    connect(
        &mQTimer, &QTimer::timeout,
        this, &SolverThread::emitProgress
    );

    mQTimer.start(timersleep_ms);
}

void
SolverThread::run() {

    // doesn't work here
    // mQTimer.start(500);

    mMS2D->solve(
        mTimeInitial,
        mTimeStep,
        mSteps + 1,
        &mCurrentStep
    );

    // Timers cannot be stopped from another thread
    // mQTimer.stop();

    // emit setProgress(mCurrentStep);
    // emit resultReady(QString("result ready"));
}

void
SolverThread::emitProgress() {
    emit setProgress(mCurrentStep);
}

int
SolverThread::getProgrees() const{
    return mCurrentStep;
}


SolversUpdateTimer::SolversUpdateTimer(
    int unsigned timersleep_ms,
    QObject *parent
):
QTimer(parent),
mTimerMs(timersleep_ms),
mThreadsList(std::vector<SolverThread*>({})),
mFinished(true),
mFinalProgess(0)
{
    setInterval(timersleep_ms);

    connect(
        this, &QTimer::timeout,
        this, &SolversUpdateTimer::solverupdate
    );

}

bool
SolversUpdateTimer::startSolvers(
    std::vector<IMS2D*> list,
    GlobalOptions::solver_options_t solver_options
)
{
    if(!mFinished){
        return false;
    }
    else{
        // nothing to be done if list is empty
        if(list.size() == 0)
            return true;

        mFinalProgess = GlobalOptions::getSteps(solver_options);

        for(std::size_t i = 0; i < list.size(); ++i){

            IMS2D* ptr = list[i];
            SolverThread *workerThread = new SolverThread(
                ptr,
                GlobalOptions::getInitialTime(solver_options),
                GlobalOptions::getDeltaTime(solver_options),
                GlobalOptions::getSteps(solver_options),
                mTimerMs
            );
            mThreadsList.push_back(workerThread);

            // threads cannot be deleted when they come to an end
            // because access to them is required at SolversUpdateTimer::solverupdate
            // connect(
            //     workerThread, &QThread::finished,
            //     workerThread, &QObject::deleteLater
            // );

            workerThread->start();
        }

        // start timer
        start();
        return true;
    }

}

void
SolversUpdateTimer::solverupdate() {

    /*
    Average progress is not a good idea
    int progress = 0;
    int N = mThreadsList.size();
    for(int i = 0; i < N; ++i){
        progress += mThreadsList[i]->getProgrees();
    }
    progress = progress / N;
    emit setProgress(progress);
    */

    /*
    Minimum progress is the correct idea
    */
    int progress = mFinalProgess;
    int N = mThreadsList.size();
    for(int i = 0; i < N; ++i){
        if(!mThreadsList[i]->isFinished())
            progress = std::min(progress, mThreadsList[i]->getProgrees());
    }
    emit setProgress(progress);

    if(progress == mFinalProgess){
        std::cout << "All solvers ended." << std::endl;
        // stop timer
        stop();
        // delete all solvers
        for(int i = 0; i < N; ++i){
            // thread should have finished, but we need to wait for run to have come to an end
            mThreadsList[0]->wait();

            delete mThreadsList[0];
            // clear vector
            mThreadsList.erase(mThreadsList.begin());
        }
    }
}
