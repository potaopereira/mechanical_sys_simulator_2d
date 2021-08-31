#pragma once

#include "msviewer/rbviewwvectors.hpp"


class RBViewEllipse:
public RBViewWVectors
{
public:
    typedef std::tuple<Eigen::Matrix<double, 2, 1>, Eigen::Matrix<double, 2, 1>, double> param_t;
    RBViewEllipse(
        Eigen::Matrix<double, 2, 1> const & center,
        Eigen::Matrix<double, 2, 1> const & axes,
        double angle = 0,
        double axesScale = 100,
        int unsigned nBodyVectors = 0,
        int unsigned nInertialVectors = 0,
        QGraphicsItem* parent = nullptr
    );
    ~RBViewEllipse();

    virtual
    void
    setBoundaryColor(
        QColor const & color
    );

    virtual
    void resetBoundary(
        void *
    );

private:
    QGraphicsEllipseItem* mEllipse;
};

class RBViewRectangle:
public RBViewWVectors
{
public:
    typedef std::tuple<Eigen::Matrix<double, 2, 1>, Eigen::Matrix<double, 2, 1>, double> param_t;
    RBViewRectangle(
        Eigen::Matrix<double, 2, 1> const & center,
        Eigen::Matrix<double, 2, 1> const & axes,
        double angle = 0,
        double axesScale = 100,
        int unsigned nBodyVectors = 0,
        int unsigned nInertialVectors = 0,
        QGraphicsItem* parent = nullptr
    );
    ~RBViewRectangle();

    virtual
    void
    setBoundaryColor(
        QColor const & color
    );

    virtual
    void resetBoundary(
        void *
    );

private:
    QGraphicsRectItem* mQGraphicsRectItem;
};

