#include "SlidingBallOnSlope/SlidingBallOnSlopeViewer.hpp"

#include <msviewer/rbwithboundary.hpp>


SlidingBallOnSlopeViewer::SlidingBallOnSlopeViewer(

):
SlidingBallOnSlopeSolver(),
MSVIEW1(
    std::array<RBViewWVectors*, 1>(
        {
            {
                // rigid body 0
                new RBViewEllipse(
                        Eigen::Matrix<double, 2, 1>(0, 0),
                        Eigen::Matrix<double, 2, 1>(getParam("radius1"), getParam("radius2")),
                        0
                    )
            }
        }
    )
),
IMS2D(this, this, this)
{

    QGraphicsLineItem* slope = new QGraphicsLineItem();
    slope->setLine(0 + 1000*(-1), 0 + 1000*(+getParam("slope")), 0 + 1000*(+1), 0 + 1000*(-getParam("slope")));
    std::vector<QGraphicsItem*> listOfConstraints = {slope};

    // MSVIEW1::setConstraints
    setConstraints(listOfConstraints);

    p0 <<  0, getParam("radius1"), 1, 0, 0, 1;
    q0 << -getParam("radius1"), 0;
    // v0 << 0.5, -0.5, 1;
    v0 << 0, 0, 0.5;

}

void
SlidingBallOnSlopeViewer::reset(){

    // rigid body 0
    RBViewEllipse::param_t param1 =
        std::make_tuple(
            Eigen::Matrix<double, 2, 1>(0, 0),
            Eigen::Matrix<double, 2, 1>(getParam("radius1"), getParam("radius2")),
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
}