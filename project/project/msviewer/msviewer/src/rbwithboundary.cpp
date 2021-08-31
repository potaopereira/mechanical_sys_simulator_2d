#include "msviewer/rbwithboundary.hpp"

RBViewEllipse::RBViewEllipse(
    Eigen::Matrix<double, 2, 1> const & center,
    Eigen::Matrix<double, 2, 1> const & axes,
    double angle,
    double axesScale,
    int unsigned nBodyVectors,
    int unsigned nInertialVectors,
    QGraphicsItem* parent
):
RBViewWVectors(
    nullptr,
    axesScale,
    nBodyVectors,
    nInertialVectors
)
,
mEllipse(nullptr)
{
    mEllipse = new QGraphicsEllipseItem(center(0) - axes(0), center(1) - axes(1), 2*axes(0), 2*axes(1));
    mEllipse->setRotation(angle);
    setBoundary(mEllipse);
}

RBViewEllipse::~RBViewEllipse(
    //
){

}

void
RBViewEllipse::setBoundaryColor(
    QColor const & color
){
    QPen pen(color, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    mEllipse->setPen(color);
}

void
RBViewEllipse::resetBoundary(
    void * paramin
){
    param_t* param = static_cast<param_t*>(paramin);
    Eigen::Matrix<double, 2, 1> center = std::get<0>(*param);
    Eigen::Matrix<double, 2, 1> axes = std::get<1>(*param);
    double angle = std::get<2>(*param);
    mEllipse->setRect(center(0) - axes(0), center(1) - axes(1), 2*axes(0), 2*axes(1));
    mEllipse->setRotation(angle);
}

RBViewRectangle::RBViewRectangle(
    Eigen::Matrix<double, 2, 1> const & center,
    Eigen::Matrix<double, 2, 1> const & axes,
    double angle,
    double axesScale,
    int unsigned nBodyVectors,
    int unsigned nInertialVectors,
    QGraphicsItem* parent
):
RBViewWVectors(
    nullptr,
    axesScale,
    nBodyVectors,
    nInertialVectors
)
,
mQGraphicsRectItem(nullptr)
{
    mQGraphicsRectItem = new QGraphicsRectItem(center(0) - axes(0)/2, center(1) - axes(1)/2, axes(0), axes(1));
    mQGraphicsRectItem->setRotation(angle);
    setBoundary(mQGraphicsRectItem);
}

RBViewRectangle::~RBViewRectangle(
    //
){

}

void
RBViewRectangle::setBoundaryColor(
    QColor const & color
){
    QPen pen(color, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    mQGraphicsRectItem->setPen(pen);
}

void
RBViewRectangle::resetBoundary(
    void * paramin
){
    param_t* param = static_cast<param_t*>(paramin);
    Eigen::Matrix<double, 2, 1> center = std::get<0>(*param);
    Eigen::Matrix<double, 2, 1> axes = std::get<1>(*param);
    double angle = std::get<2>(*param);
    mQGraphicsRectItem->setRect(center(0) - axes(0)/2, center(1) - axes(1)/2, axes(0), axes(1));
    mQGraphicsRectItem->setRotation(angle);
}