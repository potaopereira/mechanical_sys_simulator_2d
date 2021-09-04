#include "BarPendulum/BarPendulumViewer.hpp"

#include <msviewer/rbwithboundary.hpp>


BarPendulumViewer::BarPendulumViewer(

):
BarPendulumSolver(),
MSVIEW1(
    std::array<RBViewWVectors*, 1>(
        {
            {
                // rigid body 0
                new RBViewRectangle(
                        Eigen::Matrix<double, 2, 1>(0, 0),
                        Eigen::Matrix<double, 2, 1>(getParam("length1_bar")+getParam("length2_bar"), 10),
                        0
                    )
            }
        }
    )
),
IMS2D(this, this, this)
{

    point2contact1 = new QGraphicsLineItem();
    point2contact1->setLine(-100, 150, -100+0, 150+0);

    point2contact2 = new QGraphicsLineItem();
    point2contact2->setLine(+100, 150, +100+0, 150+0);

    // MSVIEW1::setConstraints
    setConstraints(listOfConstraints);

    p0 <<  0, 0, 1, 0, 0, 1;
    v0 << 0.5, -0.5, 1;
    // v0 << 0, 0, 0;

}

void
BarPendulumViewer::reset(){

    // rigid body 0
    RBViewEllipse::param_t param1 =
        std::make_tuple(
            Eigen::Matrix<double, 2, 1>(0, 0),
            Eigen::Matrix<double, 2, 1>(getParam("length1_bar"), getParam("length2_bar")),
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