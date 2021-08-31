#pragma once

#include <QGraphicsItemGroup>

#include <Eigen/Dense>

#include <memory>

#include <QPen>

class RBView:
public QGraphicsItemGroup
{
public:
    typedef Eigen::Matrix<double, 6, 1> p_t;
    typedef Eigen::Matrix<double, 2, 1> lp_t;
    typedef Eigen::Matrix<double, 2, 2> ap_t;
    typedef Eigen::Matrix<double, 2, 1> vector_t;

    RBView(
        QGraphicsItem* boundary = nullptr,
        double axesScale = 100,
        QGraphicsItem* parent = nullptr
    );

    virtual ~RBView(
    );

    void setBoundary(
        QGraphicsItem* boundary
    );

    virtual
    void
    setBoundaryColor(
        QColor const & // color
    ){

    }

    virtual
    void resetBoundary(
        void *
    ){
        
    }

    void setAxesScale(
        double axesScale
    );

    void
    setPose(
        p_t const & p // position
    );

    void setPose(
        lp_t const & linear_position
        ,
        ap_t const & angular_position
    );

    void showBoundary(
        bool show = true
    );
    void hideBoundary(
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
    double mAxesScale;
    QGraphicsLineItem* mAxes1;
    QGraphicsLineItem* mAxes2;
    QGraphicsItem* mBoundary;

    std::vector<QGraphicsLineItem*> mVectors;
};
