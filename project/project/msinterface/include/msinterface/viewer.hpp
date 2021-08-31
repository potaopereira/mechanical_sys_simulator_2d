#pragma once

// IMRB2DSOLVER
#include "msinterface/solver.hpp"

// QGraphicsItemGroup QT += widgets
#include <QGraphicsItemGroup>

// QColor QT += gui
#include <QColor>


class IMS2DViewer
{
public:

    typedef IMS2DSolver::rbp_t rbp_t;
    typedef IMS2DSolver::rbv_t rbv_t;
    typedef IMS2DSolver::rbf_t rbf_t;
    typedef IMS2DSolver::v_t v_t;

    IMS2DViewer(){}

    void setSolver(
        IMS2DSolver* solver
    ){
        mSolver = solver;
    }

    void showAtStep(
        int step
    );

    void show(
        int step,
        void (IMS2DViewer::*show_ptr)(int step)
    );

    void showAll(
        int step
    );

    void showPositions(
        int step
    );

    void showVelocities(
        int step
    );

    void showForces(
        int step
    );


    /**************************************************************************/
    /* pure virtual methods                                                   */
    /**************************************************************************/

    virtual
    void setPoseAndVectorsOfRB(
        rbp_t const & position
        ,
        rbv_t velocity
        ,
        rbf_t force
        ,
        std::vector<v_t> const & inertial_vectors
        ,
        std::vector<v_t> const & body_vectors
        ,
        int body_id
    ) = 0;

    virtual
    void setPosition(
        rbp_t const & position
        ,
        int body_id
    ) = 0;

    virtual
    void setVelocity(
        rbv_t const & velocity
        ,
        int body_id
    ) = 0;

    virtual
    void setForce(
        rbf_t const & force
        ,
        int body_id
    ) = 0;

    virtual
    void showBoundary(
        int i, // rigid body id
        bool show
    ) = 0;

    virtual
    void setBoundaryColor(
        int i, // rigid body id
        QColor
    ) = 0;

    virtual
    void reset(

    ) = 0;

    virtual
    QGraphicsItemGroup* getView() = 0;

private:
    IMS2DSolver* mSolver;
};
