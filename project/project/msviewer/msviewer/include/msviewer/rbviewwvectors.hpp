#pragma once

// RBView
#include "msviewer/rbview.hpp"

#include <QGraphicsItem>

#include <QGraphicsLineItem>

#include <QColor>

class RBViewWVectors:
public QGraphicsItemGroup
{
public:
    typedef Eigen::Matrix<double, 2, 1> vector_t;

    RBViewWVectors(
        QGraphicsItem* boundary = nullptr,
        double axesScale = 100,
        int unsigned nBodyVectors = 0,
        int unsigned nInertialVectors = 0,
        QGraphicsItem* parent = nullptr
    );

    virtual ~RBViewWVectors(
    );

    void setBoundary(
        QGraphicsItem* boundary
    );

    void showBoundary(
        bool show
    );

    virtual
    void
    setBoundaryColor(
        QColor const & color
    );

    virtual
    void resetBoundary(
        void *
    );

    void setAxesScale(
        double axesScale
    );

    void setPose(
        Eigen::Matrix<double, 6, 1> const & position
    );

    void setPose(
        Eigen::Matrix<double, 2, 1> const & p // linear_position
        ,
        Eigen::Matrix<double, 2, 2> const & r // angular_position
    );

    void setPoseAndVectors(
        Eigen::Matrix<double, 6, 1> const & position
        ,
        Eigen::Matrix<double, 3, 1> velocity
        ,
        Eigen::Matrix<double, 3, 1> force
        ,
        std::vector<Eigen::Matrix<double, 2, 1>> const & inertial_vectors
        ,
        std::vector<Eigen::Matrix<double, 2, 1>> const & body_vectors
    );

    void showPosition(
    );
    void hidePosition(
    );
    void setPosition(
        Eigen::Matrix<double, 6, 1> const & position = Eigen::Matrix<double, 6, 1>::Zero()
    );

    void showVelocity(
    );
    void hideVelocity(
    );
    void setVelocity(
        Eigen::Matrix<double, 3, 1> const & velocity = Eigen::Matrix<double, 3, 1>::Zero()
    );

    void showLinearVelocity(
    );
    void hideLinearVelocity(
    );
    void setLinearVelocity(
        Eigen::Matrix<double, 2, 1> const & linear_velocity = Eigen::Matrix<double, 2, 1>::Zero()
    );

    void showAngularVelocity(
    );
    void hideAngularVelocity(
    );
    void setAngularVelocity(
        double const & angular_velocity = 0
    );

    void showForce(
    );
    void hideForce(
    );
    void setForce(
        Eigen::Matrix<double, 3, 1> const & force = Eigen::Matrix<double, 3, 1>::Zero()
    );

    void showLinearForce(
    );
    void hideLinearForce(
    );
    void setLinearForce(
        Eigen::Matrix<double, 2, 1> const & linear_force = Eigen::Matrix<double, 2, 1>::Zero()
    );

    void showAngularForce(
    );
    void hideAngularForce(
    );
    void setAngularForce(
        double const & angular_force = 0
    );

    void addVector(
        QPen const & pen = QPen(QColor(0, 0, 0), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin)
    );
    void showVector(
        int unsigned n
    );
    void hideVector(
        int unsigned n
    );
    void setVector(
        int unsigned n,
        vector_t const & vector
    );

private:
    RBView* mRBView;
    QGraphicsItemGroup* mGroupInertialVectors;
    std::vector<QGraphicsLineItem*> mInertialVectors;
    // QGraphicsLineItem* mLinearVelocity;
    // QGraphicsLineItem* mLinearForce;
    // QGraphicsLineItem* mAngularVelocity;
    // QGraphicsLineItem* mAngularForce;

};
