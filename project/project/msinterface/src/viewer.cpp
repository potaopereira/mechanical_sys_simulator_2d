#include "msinterface/viewer.hpp"


void
IMS2DViewer::showAtStep(
    int step
){
    for(int body_id = 0; body_id < mSolver->dimRigidBodies(); ++body_id){
        setPoseAndVectorsOfRB(
            mSolver->getPositionPlot(step, body_id),
            mSolver->getVelocityPlot(step, body_id),
            mSolver->getForcePlot(step, body_id),
            mSolver->getInertialVectorsPlot(step, body_id),
            mSolver->getBodyVectorsPlot(step, body_id),
            body_id
        );
    }
}

void
IMS2DViewer::show(
    int step,
    void (IMS2DViewer::*show_ptr)(int step)
){
    (this->*show_ptr)(step);
}

void
IMS2DViewer::showAll(
    int step
){
    showPositions(step);
    showVelocities(step);
    showForces(step);
}

void
IMS2DViewer::showPositions(
    int step
){
    for(int body_id = 0; body_id < mSolver->dimRigidBodies(); ++body_id){
        setPosition(
            mSolver->getPositionPlot(step, body_id),
            body_id
        );
    }
}

void
IMS2DViewer::showVelocities(
    int step
){
    for(int body_id = 0; body_id < mSolver->dimRigidBodies(); ++body_id){
        setVelocity(
            mSolver->getVelocityPlot(step, body_id),
            body_id
        );
    }
}

void
IMS2DViewer::showForces(
    int step
){
    for(int body_id = 0; body_id < mSolver->dimRigidBodies(); ++body_id){
        setForce(
            mSolver->getForcePlot(step, body_id),
            body_id
        );
    }
}
