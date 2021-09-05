#pragma once

// RBView
#include "msviewer/rbview.hpp"

#include <QGraphicsItem>

#include <QGraphicsLineItem>

#include <QColor>

#include <QPolygonF>
#include <QPainterPath>
#include <QGraphicsPathItem>
#include <QVector>

class PointPath:
public QGraphicsPathItem
{
public:
    typedef Eigen::Matrix<double, 2, 1> vector_t;

    PointPath(
        int unsigned length,
        QGraphicsItem* parent = nullptr
    ):
    QGraphicsPathItem(parent)
    ,
    mPointPathLen(length)
    ,
    mPointPathPoints(QVector<QPointF>({}))
    {
        // setPath();
    }


    void showPointPath(
        bool show,
        int unsigned length
    ){

        setVisible(show);
        mPointPathLen = length;
        mPointPathPoints.clear();


    }

    void addToPointPath(
        std::array<double, 2> point
    ){
        // @todo: very inneficient approach
        mPointPathPoints.push_back(QPointF(point[0], point[1]));
        if(mPointPathPoints.size() < mPointPathLen)
            ;
        else
            mPointPathPoints.erase(mPointPathPoints.begin());

        QPolygonF polygon(mPointPathPoints);
        QPainterPath path;
        path.addPolygon(polygon);
        setPath(path);

    }

    void setLength(
        int unsigned length
    ){
        mPointPathLen = length;
        mPointPathPoints.clear();
    }

    void show(
        bool show
    ){
        setVisible(true);
        if(show)
            mPointPathPoints.clear();
    }

private:
    int mPointPathLen;
    QVector<QPointF> mPointPathPoints;
};

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

    void showRelativePoint(
        bool show
    );

    void setRelativePoint(
        std::array<double, 2> point
    );

    void showVectorAtPoint(
        bool show
    );

    void setVectorAtPoint(
        std::array<double, 2> point,
        std::array<double, 2> velocity
    );

    void showPointPath(
        bool show,
        int unsigned length = 0
    );

    void addToPointPath(
        std::array<double, 2> point
    );

private:
    RBView* mRBView;
    QGraphicsItemGroup* mGroupInertialVectors;
    std::vector<QGraphicsLineItem*> mInertialVectors;
    QGraphicsEllipseItem* mRelativePoint;
    QGraphicsLineItem* mVectorAtPoint;
    PointPath* mPointPath;
    // QGraphicsLineItem* mLinearVelocity;
    // QGraphicsLineItem* mLinearForce;
    // QGraphicsLineItem* mAngularVelocity;
    // QGraphicsLineItem* mAngularForce;

};
