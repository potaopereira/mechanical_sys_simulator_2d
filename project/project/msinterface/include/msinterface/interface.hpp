#pragma once

// IMRB2DPARAM
#include "msinterface/param.hpp"

// IMS2DSolver
#include "msinterface/solver.hpp"

// IMS2DViewer
#include "msinterface/viewer.hpp"

class IMS2D
{
public:

    IMS2D(
        IMS2DSolver * solver,
        IMS2DViewer * viewer,
        IMRB2DPARAM * param
    ):
    mSolver(solver),
    mViewer(viewer),
    mParam(param)
    {
        // link viewer to solver
        mViewer->setSolver(mSolver);
    }

    IMRB2DPARAM::map_t& getFloatParameters(){
        return mParam->getFloatParameters();
    };

    void setFloatParameter(std::string paraName, float value){
        mParam->setParam(paraName, value);
        mViewer->reset();
    };

    void showAtStep(
        int step
    ){
        mViewer->showAtStep(step);
    }

    int solve(
        double time_initial,
        double time_step,
        int steps,
        int * current_step
    ){
        return mSolver->solve(
            time_initial,
            time_step,
            steps,
            current_step
        );
    }

    int dimRigidBodies(){
        return mSolver->dimRigidBodies();
    }

    std::array<double, 3> getPosition(
        int step,
        int N // rigid body number
    ) const {
        return mSolver->getPosition(step, N);
    };

    std::array<double, 3> getVelocity(
        int step,
        int N // rigid body number
    ) const {
        return mSolver->getVelocity(step, N);
    };

    std::array<double, 3> getForce(
        int step,
        int N // rigid body number
    ) const {
        return mSolver->getForce(step, N);
    };

    virtual
    std::array<double, 2> getPositionOfPoint(
        int step,
        int N, // rigid body number
        std::array<double, 2> relative_position
    ) const {
        return mSolver->getPositionOfPoint(step, N, relative_position);
    };

    virtual
    std::array<double, 2> getVelocityOfPoint(
        int step,
        int N, // rigid body number
        std::array<double, 2> relative_position
    ) const {
        return mSolver->getVelocityOfPoint(step, N, relative_position);
    };

    virtual
    QGraphicsItemGroup* getView(){
        return mViewer->getView();
    };

    void showBoundary(
        int i, // rigid body id
        bool show
    ){
        mViewer->showBoundary(i, show);
    };

    void setBoundaryColor(
        int i, // rigid body id
        QColor color
    ){
        mViewer->setBoundaryColor(i, color);
    }

private:
    IMS2DSolver * mSolver;
    IMS2DViewer * mViewer;
    IMRB2DPARAM * mParam;
};