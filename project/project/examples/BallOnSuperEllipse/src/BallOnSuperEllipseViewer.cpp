#include "BallOnSuperEllipse/BallOnSuperEllipseViewer.hpp"

#include <msviewer/rbwithboundary.hpp>

#include <QPainterPath>
#include <QPolygonF>
#include <cmath>

BallOnSuperEllipseViewer::BallOnSuperEllipseViewer(

):
BallOnSuperEllipseSolver(),
MSVIEW1(
    std::array<RBViewWVectors*, 1>(
        {
            {
                // rigid body 0
                new RBViewEllipse(
                        Eigen::Matrix<double, 2, 1>(0, 0),
                        Eigen::Matrix<double, 2, 1>(getParam("ball_r"), getParam("ball_r")),
                        0
                    )
            }
        }
    )
),
IMS2D(this, this, this)
,
mListOfConstraints(std::vector<QGraphicsItem*>({}))
,
mSuperEllipse(new QGraphicsPathItem())
{

    QPolygonF myPolygon;
    int k = 200;
    double r = 1./getParam("outside_a");
    double alpha = getParam("outside_alpha");
    for(int i = 0; i < k + 1; ++i){
        double p = i*2*M_PI/k;
        double x = cos(p) >= 0 ? r*pow(cos(p), 2/alpha) : -r*pow(-cos(p), 2/alpha);
        double y = sin(p) >= 0 ? r*pow(sin(p), 2/alpha) : -r*pow(-sin(p), 2/alpha);
        myPolygon << QPointF(x, y);
    }

    QPainterPath path;
    path.addPolygon(myPolygon);
    mSuperEllipse->setPath(path);

    mListOfConstraints.push_back(mSuperEllipse);

    // MSVIEW1::setConstraints
    setConstraints(mListOfConstraints);

    p0 << -1./getParam("outside_a"), 0, 1, 0, 0, 1;
    v0 << 0, 0, 0;

}

void
BallOnSuperEllipseViewer::reset(){

    // rigid body 0
    RBViewEllipse::param_t param1 =
        std::make_tuple(
            Eigen::Matrix<double, 2, 1>(0, 0),
            Eigen::Matrix<double, 2, 1>(getParam("ball_r"), getParam("ball_r")),
            0
        );

    // MSVIEW1
    MSVIEW1::reset(
        std::array<void*, 1>(
            {
                {
                    &param1
                }
            }
        )
    );

    QPolygonF myPolygon;
    int k = 200;
    double r = 1./getParam("outside_a");
    double alpha = getParam("outside_alpha");
    for(int i = 0; i < k + 1; ++i){
        double p = i*2*M_PI/k;
        double x = cos(p) >= 0 ? r*pow(cos(p), 2/alpha) : -r*pow(-cos(p), 2/alpha);
        double y = sin(p) >= 0 ? r*pow(sin(p), 2/alpha) : -r*pow(-sin(p), 2/alpha);
        myPolygon << QPointF(x, y);
    }

    QPainterPath path;
    path.addPolygon(myPolygon);
    mSuperEllipse->setPath(path);
}