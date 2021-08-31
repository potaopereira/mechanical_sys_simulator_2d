#include "msviewer/rbview.hpp"

#include <Qt>

// Qt+=gui
// QColor
#include <QColor>

// QPen
#include <QPen>

// atan2
#include <cmath>

/*
memory allocated by new is not deallocated here

RBView will eventually be added to a Scene, which will do the deallocation
*/

RBView::RBView(
    QGraphicsItem* boundary
    ,
    double axesScale
    ,
    QGraphicsItem* parent
):
QGraphicsItemGroup(parent)
,
mAxesScale(axesScale)
,
mAxes1(new QGraphicsLineItem(0, 0, mAxesScale, 0))
,
mAxes2(new QGraphicsLineItem(0, 0, 0, mAxesScale))
,
mBoundary(boundary)
,
mVectors(std::vector<QGraphicsLineItem*>({}))
{

    // QPen pen1(Qt::red, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    // red: Hex: #FF0000 RGBA(255, 0, 0, 1)
    // dark red: Hex: #8B0000 RGBA(139, 0, 0, 1)
    QPen pen1(QColor(139, 0, 0), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    mAxes1->setPen(pen1);
    addToGroup(mAxes1);

    // QPen pen2(Qt::green, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    // QColor(int r, int g, int b, int a = 255);
    // dark green: Hex: #006400 RGBA(0, 100, 0, 1)
    QPen pen2(QColor(0, 100, 0), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    mAxes2->setPen(pen2);
    addToGroup(mAxes2);

    if(mBoundary!=nullptr)
        addToGroup(mBoundary);
}

RBView::~RBView(){
}

void
RBView::setBoundary(
    QGraphicsItem* boundary
){
    if(mBoundary==nullptr){
        mBoundary = boundary;
        addToGroup(mBoundary);
    }
}

void
RBView::setAxesScale(
    double axesScale
){
    mAxesScale = axesScale;
    mAxes1->setLine(0, 0, mAxesScale, 0);
    mAxes2->setLine(0, 0, 0, mAxesScale);
}

void
RBView::setPose(
    p_t const & p // position
){
    this->setPos(p(0), p(1));
    double angle = std::atan2(p(4), p(2)) * 180/M_PI;
    this->setRotation(angle);
}


void
RBView::setPose(
    lp_t const & p // linear_position
    ,
    ap_t const & r // angular_position
){
    this->setPos(p(0),p(1));
    double angle = std::atan2(r(1,0), r(0,0)) * 180/M_PI;
    this->setRotation(angle);
}

void
RBView::showBoundary(
    bool show
){
    if(mBoundary!=nullptr)
        mBoundary->setVisible(show);
}

void
RBView::hideBoundary(
){
    if(mBoundary!=nullptr)
        mBoundary->setVisible(false);
}

void
RBView::addVector(
    QPen const & pen
){
    QGraphicsLineItem* p = new QGraphicsLineItem(0, 0, 0, 0);
    p->setPen(pen);
    mVectors.push_back(p);
    addToGroup(p);
}

void
RBView::showVector(
    int unsigned n
){
    if(n < static_cast<int unsigned>(mVectors.size()))
        mVectors[n]->setVisible(true);
}

void
RBView::hideVector(
    int unsigned n
){
    if(n < static_cast<int unsigned>(mVectors.size()))
        mVectors[n]->setVisible(false);
}

void
RBView::setVector(
    int unsigned n,
    vector_t const & vector
){
    if(n < static_cast<int unsigned>(mVectors.size()))
        mVectors[n]->setLine(0, 0, vector(0), vector(1));
}
