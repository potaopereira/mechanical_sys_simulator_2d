#include "BallOnCircle/BallOnCircleViewer.hpp"

#include <msviewer/rbwithboundary.hpp>

BallOnCircleViewer::BallOnCircleViewer(

):
BallOnCircleSolver(),
MSVIEW1(
    std::array<RBViewWVectors*, 1>(
        {
            {
                // rigid body 0
                new RBViewEllipse(
                        Eigen::Matrix<double, 2, 1>(getParam("ball_offset0"), getParam("ball_offset1")),
                        Eigen::Matrix<double, 2, 1>(getParam("ball_a0"), getParam("ball_a1")),
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
mEllipse(nullptr)
{

    mEllipse = new QGraphicsEllipseItem(0 - getParam("outside_a0"), 0 - getParam("outside_a1"), 2*getParam("outside_a0"), 2*getParam("outside_a1"));
    mListOfConstraints.push_back(mEllipse);

    // MSVIEW1::setConstraints
    setConstraints(mListOfConstraints);

    p0 << getParam("outside_a0")/2, 0, 1, 0, 0, 1;
    q0 << getParam("ball_a0"), 0;
    v0 << 0, 0, 0;

}

void
BallOnCircleViewer::reset(){

    // rigid body 0
    RBViewEllipse::param_t param1 =
        std::make_tuple(
            Eigen::Matrix<double, 2, 1>(getParam("ball_offset0"), getParam("ball_offset1")),
            Eigen::Matrix<double, 2, 1>(getParam("ball_a0"), getParam("ball_a1")),
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

    mEllipse->setRect(0 - getParam("outside_a0"), 0 - getParam("outside_a1"), 2*getParam("outside_a0"), 2*getParam("outside_a1"));
}