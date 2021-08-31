#include "msinterface/solver.hpp"


IMS2DSolver::IMS2DSolver(

):
mAllocated(false)
,
mSolutions(nullptr)
{

}

IMS2DSolver::~IMS2DSolver(){
    deallocate();
}


int
IMS2DSolver::solve(
    double time_initial,
    double time_step,
    int steps,
    int * current_step
){
    allocate(steps);

    return solverImplementation(
        time_initial,
        time_step,
        mSteps,
        mSolutions,
        current_step
    );
}


bool
IMS2DSolver::allocate(
    int unsigned steps
){
    deallocate();

    int N = dimSolution();
    mSteps = steps;
    mSolutions = new double*[mSteps];
    for(int i = 0; i < mSteps; ++i)
        mSolutions[i] = new double[N];

    mAllocated = true;
    
    return true;
}

bool
IMS2DSolver::deallocate(

){
    if(mAllocated){
        //Free each sub-array
        for(int i = 0; i < mSteps; ++i)
            delete[] mSolutions[i];
        //Free the array of pointers
        delete[] mSolutions;
        mAllocated = false;
    }
    return true;
}

/**************************************************************************/
/* for plotting I decided to use eigen                                    */
/**************************************************************************/

IMS2DSolver::rbp_t IMS2DSolver::getPositionPlot(
    int step,
    int N // rigid body number
)const {
    if(step < mSteps){
        double* solution = mSolutions[step];
        return getPositionPlot(solution, N);
    }
    return IMS2DSolver::rbp_t::Zero();
}

IMS2DSolver::rbv_t IMS2DSolver::getVelocityPlot(
    int step,
    int N // rigid body number
)const {
    if(step < mSteps){
        double* solution = mSolutions[step];
        return getVelocityPlot(solution, N);
    }
    return IMS2DSolver::rbv_t::Zero();
}

IMS2DSolver::rbf_t IMS2DSolver::getForcePlot(
    int step,
    int N // rigid body number
)const {
    if(step < mSteps){
        double* solution = mSolutions[step];
        return getForcePlot(solution, N);
    }
    return IMS2DSolver::rbf_t::Zero();
}

std::vector<IMS2DSolver::v_t> IMS2DSolver::getInertialVectorsPlot(
    int step,
    int N // rigid body number
)const {
    if(step < mSteps){
        double* solution = mSolutions[step];
        return getInertialVectorsPlot(solution, N);
    }
    return std::vector<v_t>({});
}

std::vector<IMS2DSolver::v_t> IMS2DSolver::getBodyVectorsPlot(
    int step,
    int N // rigid body number
)const {
    if(step < mSteps){
        double* solution = mSolutions[step];
        return getBodyVectorsPlot(solution, N);
    }
    return std::vector<v_t>({});
}

/**************************************************************************/
/* for showing numbers  I decided to use arrays                           */
/**************************************************************************/

std::array<double, 3> IMS2DSolver::getPosition(
    int step,
    int N // rigid body number
)const {
    if(step < mSteps){
        double* solution = mSolutions[step];
        return getPosition(solution, N);
    }
    return std::array<double, 3>({{0,0,0}});
}

std::array<double, 3> IMS2DSolver::getVelocity(
    int step,
    int N // rigid body number
)const {
    if(step < mSteps){
        double* solution = mSolutions[step];
        return getVelocity(solution, N);
    }
    return std::array<double, 3>({{0,0,0}});
}

std::array<double, 3> IMS2DSolver::getForce(
    int step,
    int N // rigid body number
)const {
    if(step < mSteps){
        double* solution = mSolutions[step];
        return getForce(solution, N);
    }
    return std::array<double, 3>({{0,0,0}});
}

std::array<double, 2> IMS2DSolver::getPositionOfPoint(
    int step,
    int N, // rigid body number
    std::array<double, 2> relative_position
)const {
    if(step < mSteps){
        double* solution = mSolutions[step];
        return getPositionOfPoint(solution, N, relative_position);
    }
    return std::array<double, 2>({{0,0}});
}

std::array<double, 2> IMS2DSolver::getVelocityOfPoint(
    int step,
    int N, // rigid body number
    std::array<double, 2> relative_position
)const {
    if(step < mSteps){
        double* solution = mSolutions[step];
        return getVelocityOfPoint(solution, N, relative_position);
    }
    return std::array<double, 2>({{0,0}});
}

