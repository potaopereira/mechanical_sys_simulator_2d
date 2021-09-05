#pragma once

// IMRB2DPARAM
#include "msinterface/param.hpp"

// IMS2DSolver
#include "msinterface/solver.hpp"

// IMS2DViewer
#include "msinterface/viewer.hpp"

// std::tuple
#include <tuple>

class IMS2D
{
public:

    typedef std::vector< std::tuple<bool, std::array<double, 2>>> show_relative_t;

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

        for(int i = 0; i < dimRigidBodies(); ++i){
            mShowRelativePoint.push_back(
                std::make_tuple(false, std::array<double, 2>({{0,0}}))
            );
            mShowRelativePointVelocity.push_back(
                std::make_tuple(false, std::array<double, 2>({{0,0}}))
            );
            mShowRelativePointPath.push_back(
                std::make_tuple(false, std::array<double, 2>({{0,0}}))
            );
        }
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
        
        for(int i = 0; i < dimRigidBodies(); ++i){
            if(std::get<0>(mShowRelativePoint[i]))
                mViewer->setRelativePoint(
                    i,
                    getPositionOfPoint(step, i, std::get<1>(mShowRelativePoint[i]))
                );
            if(std::get<0>(mShowRelativePointVelocity[i]))
                mViewer->setRelativePointVelocity(
                    i,
                    getPositionOfPoint(step, i, std::get<1>(mShowRelativePointVelocity[i])),
                    getVelocityOfPoint(step, i, std::get<1>(mShowRelativePointVelocity[i]))
                );
            if(std::get<0>(mShowRelativePointPath[i]))
                mViewer->setRelativePointPath(
                    i, 
                    getPositionOfPoint(step, i, std::get<1>(mShowRelativePointPath[i]))
                );
        }
    }

    std::string getName() const {
        return mSolver->getName();
    }

    std::string getName(
        int N // rigid body number
    ) const {
        return mSolver->getName(N);
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

    typedef std::tuple<std::array<double, 2>, std::array<double, 2>> plot_factors_t;
    void setPlotFactors(
        plot_factors_t plot_factors
    ) {
        setPlotFactors(
            std::get<0>(plot_factors),
            std::get<1>(plot_factors)
        );
    };
    void setPlotFactors(
        std::array<double, 2> velocity_factors,
        std::array<double, 2> force_factors
    ) {
        mSolver->setVelocityPlotFactor(velocity_factors);
        mSolver->setForcePlotFactor(force_factors);
    };

    double getLinearInertia(
        int N // rigid body number
    ) const{
        return mSolver->getLinearInertia(N);
    };

    void setLinearInertia(
        double mass,
        int N // rigid body number
    ){
        mSolver->setLinearInertia(mass, N);
    }

    double getAngularInertia(
        int N // rigid body number
    ) const {
        return mSolver->getAngularInertia(N);
    }

    void setAngularInertia(
        double moment_of_inertia,
        int N // rigid body number
    ){
        mSolver->setAngularInertia(moment_of_inertia, N);
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

    void showRelativePoint(
        bool show,
        int N, // rigid body number
        std::array<double, 2> relative_position
    ) {
        mViewer->showRelativePoint(show, N);
        mShowRelativePoint[N] = std::make_tuple(
            show,
            relative_position
        );
    };

    void showRelativePointVelocity(
        bool show,
        int N, // rigid body number
        std::array<double, 2> relative_position
    ) {
        mViewer->showRelativePointVelocity(show, N);
        mShowRelativePointVelocity[N] = std::make_tuple(
            show,
            relative_position
        );
    };

    void showRelativePointPath(
        bool show,
        int N, // rigid body number
        std::array<double, 2> relative_position,
        int unsigned length
    ) {
        mViewer->showRelativePointPath(show, N, length);
        mShowRelativePointPath[N] = std::make_tuple(
            show,
            relative_position
        );
    };

    virtual
    QGraphicsItemGroup* getView(){
        return mViewer->getView();
    };

    void showPosition(
        int i, // rigid body id
        bool show
    ){
        mViewer->showPosition(i, show);
    };

    void showVelocity(
        int i, // rigid body id
        bool show
    ){
        mViewer->showVelocity(i, show);
    }

    void showForce(
        int i, // rigid body id
        bool show
    ){
        mViewer->showForce(i, show);
    }

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

    show_relative_t mShowRelativePoint;
    show_relative_t mShowRelativePointVelocity;
    show_relative_t mShowRelativePointPath;

};