#include "msviewer/rbviewwvectors.hpp"

#include <Qt>

// Qt+=gui
// QColor
#include <QColor>

// QPen
#include <QPen>

// atan2
#include <cmath>


RBViewWVectors::RBViewWVectors(
    QGraphicsItem* boundary,
    double axesScale,
    int unsigned nBodyVectors,
    int unsigned nInertialVectors,
    QGraphicsItem* parent
):
QGraphicsItemGroup(parent)
,
mRBView(
    new
    RBView(
        boundary,
        axesScale,
        parent
    )
)
,
mGroupInertialVectors(new QGraphicsItemGroup())
,
mInertialVectors(std::vector<QGraphicsLineItem*>({}))
,
mRelativePoint(new QGraphicsEllipseItem())
,
mVectorAtPoint(new QGraphicsLineItem())
,
mPointPath(new PointPath(1))
{
    // angular velocity: purple
    mRBView->addVector(QPen(QColor(128, 0, 128), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    // angular force: purple
    mRBView->addVector(QPen(QColor(128, 0, 128), 3, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin));
    
    for(int unsigned i = 0; i < nBodyVectors; ++i)
        mRBView->addVector();

    addToGroup(mRBView);

    // linear velocity: orange
    addVector(QPen(QColor(255, 165, 0), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    // linear force: orange
    addVector(QPen(QColor(255, 165, 0), 3, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin));

    for(int unsigned i = 0; i < nInertialVectors; ++i)
        addVector();

    addToGroup(mGroupInertialVectors);

    showRelativePoint(false);
    addToGroup(mRelativePoint);
    showVectorAtPoint(false);
    addToGroup(mVectorAtPoint);
    showPointPath(false);
    addToGroup(mPointPath);

}

RBViewWVectors::~RBViewWVectors(){
}

void RBViewWVectors::setBoundary(
    QGraphicsItem* boundary
){
    mRBView->setBoundary(boundary);
}

void RBViewWVectors::showBoundary(
    bool show
){
    mRBView->showBoundary(show);
}


void
RBViewWVectors::setBoundaryColor(
    QColor const & color
){
    mRBView->setBoundaryColor(color);
}

void
RBViewWVectors::resetBoundary(
    void * param
){
    mRBView->resetBoundary(param);
}

void
RBViewWVectors::setAxesScale(
    double axesScale
){
    mRBView->setAxesScale(axesScale);
}

void
RBViewWVectors::setPose(
    Eigen::Matrix<double, 6, 1> const & p // position
){
    mRBView->setPose(p);
    mGroupInertialVectors->setPos(p(0), p(1));
}

void
RBViewWVectors::setPose(
    Eigen::Matrix<double, 2, 1> const & p // linear_position
    ,
    Eigen::Matrix<double, 2, 2> const & r // angular_position
){
    mRBView->setPose(p, r);
    mGroupInertialVectors->setPos(p(0), p(1));
}

void
RBViewWVectors::setPoseAndVectors(
    Eigen::Matrix<double, 6, 1> const & position
    ,
    Eigen::Matrix<double, 3, 1> velocity
    ,
    Eigen::Matrix<double, 3, 1> force
    ,
    std::vector<Eigen::Matrix<double, 2, 1>> const & inertial_vectors
    ,
    std::vector<Eigen::Matrix<double, 2, 1>> const & body_vectors
){
    setPose(position);

    setVelocity(velocity);
    setForce(force);

    for(std::size_t i = 0; i < inertial_vectors.size(); ++i)
        setVector(2 + i, inertial_vectors[i]);

    for(std::size_t i = 0; i < body_vectors.size(); ++i)
        mRBView->setVector(2 + i, body_vectors[i]);

}

void
RBViewWVectors::showPosition(
){
    mRBView->showPosition();
}

void
RBViewWVectors::hidePosition(
){
    mRBView->hidePosition();
}

void
RBViewWVectors::setPosition(
    Eigen::Matrix<double, 6, 1> const & position
){
    mRBView->setPose(position);
}

void
RBViewWVectors::showVelocity(
){
    showLinearVelocity();
    showAngularVelocity();
}

void RBViewWVectors::hideVelocity(
){
    hideLinearVelocity();
    hideAngularVelocity();
}

void
RBViewWVectors::setVelocity(
    Eigen::Matrix<double, 3, 1> const & velocity
){
    setLinearVelocity(velocity.segment(0,2));
    setAngularVelocity(velocity(2));
}

void
RBViewWVectors::showLinearVelocity(
){
    mInertialVectors[0]->setVisible(true);
}

void
RBViewWVectors::hideLinearVelocity(
){
    mInertialVectors[0]->setVisible(false);
}

void
RBViewWVectors::setLinearVelocity(
    Eigen::Matrix<double, 2, 1> const & v // linear_velocity
){
    mInertialVectors[0]->setLine(0, 0, v(0), v(1));
}

void
RBViewWVectors::showAngularVelocity(
){
    mRBView->showVector(0);
}

void
RBViewWVectors::hideAngularVelocity(
){
    mRBView->hideVector(0);
}

void
RBViewWVectors::setAngularVelocity(
    double const & v // angular_velocity
){
    mRBView->setVector(0, vector_t({v,v}));
}


void
RBViewWVectors::showForce(
){
    showLinearForce();
    showAngularForce();
}

void RBViewWVectors::hideForce(
){
    hideLinearForce();
    hideAngularForce();
}

void
RBViewWVectors::setForce(
    Eigen::Matrix<double, 3, 1> const & force
){
    setLinearForce(force.segment(0,2));
    setAngularForce(force(2));
}

void
RBViewWVectors::showLinearForce(
){
    mInertialVectors[1]->setVisible(true);
}

void
RBViewWVectors::hideLinearForce(
){
    mInertialVectors[1]->setVisible(false);
}

void
RBViewWVectors::setLinearForce(
    Eigen::Matrix<double, 2, 1> const & v // linear_force
){
    mInertialVectors[1]->setLine(0, 0, v(0), v(1));
}

void
RBViewWVectors::showAngularForce(
){
    mRBView->showVector(1);
}

void
RBViewWVectors::hideAngularForce(
){
    mRBView->hideVector(1);
}

void
RBViewWVectors::setAngularForce(
    double const & v // angular_force
){
    mRBView->setVector(1, vector_t({v,v}));
}

void
RBViewWVectors::addVector(
    QPen const & pen
){
    QGraphicsLineItem* p = new QGraphicsLineItem(0, 0, 0, 0);
    p->setPen(pen);
    mInertialVectors.push_back(p);
    mGroupInertialVectors->addToGroup(p);
}

void
RBViewWVectors::showVector(
    int unsigned n
){
    if(n < static_cast<int unsigned>(mInertialVectors.size()))
        mInertialVectors[n]->setVisible(true);
}

void
RBViewWVectors::hideVector(
    int unsigned n
){
    if(n < static_cast<int unsigned>(mInertialVectors.size()))
        mInertialVectors[n]->setVisible(false);
}

void
RBViewWVectors::setVector(
    int unsigned n,
    vector_t const & vector
){
    if(n < static_cast<int unsigned>(mInertialVectors.size()))
        mInertialVectors[n]->setLine(0, 0, vector(0), vector(1));
}

void RBViewWVectors::showRelativePoint(
    bool show
){
    QPen pen(QColor(0, 0, 255), 4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    mRelativePoint->setPen(pen);
    mRelativePoint->setVisible(show);
}

void RBViewWVectors::setRelativePoint(
    std::array<double, 2> point
){
    mRelativePoint->setRect(point[0] - 1, point[1] - 1, 2, 2);
}

void RBViewWVectors::showVectorAtPoint(
    bool show
){
    QPen pen(QColor(0, 0, 255), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    mVectorAtPoint->setPen(pen);
    mVectorAtPoint->setVisible(show);
}

void RBViewWVectors::setVectorAtPoint(
    std::array<double, 2> point,
    std::array<double, 2> velocity
){
    mVectorAtPoint->setLine(point[0], point[1], point[0] + velocity[0], point[1] + velocity[1]);
}

void RBViewWVectors::showPointPath(
    bool show,
    int unsigned length
){
    mPointPath->setVisible(show);
    mPointPath->setLength(length);
}

void RBViewWVectors::addToPointPath(
    std::array<double, 2> point
){
    mPointPath->addToPointPath(point);
}