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
    point2contact1->setLine(getParam("p1[0]"), getParam("p1[1]"), getParam("p1[0]"), getParam("p1[1]"));

    point2contact2 = new QGraphicsLineItem();
    point2contact2->setLine(getParam("p2[0]"), getParam("p2[1]"), getParam("p2[0]"), getParam("p2[1]"));

    std::vector<QGraphicsItem*> listOfConstraints = {point2contact1, point2contact2};

    // MSVIEW1::setConstraints
    setConstraints(listOfConstraints);

    p0 <<  0, 0, 1, 0, 0, 1;
    v0 << 400, 0, 0;
    // v0 << 0, 0, 0;

}

void
BarPendulumViewer::reset(){

    // rigid body 0
    RBViewEllipse::param_t param1 =
        std::make_tuple(
            Eigen::Matrix<double, 2, 1>(0, 0),
            Eigen::Matrix<double, 2, 1>(getParam("length1_bar")+getParam("length2_bar"), 10),
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

// over
void
BarPendulumViewer::showExtraAtStep(
    int step
){

    std::array<double, 2> p1 = IMS2DSolver::getPositionOfPoint(
        step, 0, std::array<double, 2>({{-getParam("length1_bar"), 0}})
    );
    point2contact1->setLine(getParam("p1[0]"), getParam("p1[1]"), p1[0], p1[1]);

    std::array<double, 2> p2 = IMS2DSolver::getPositionOfPoint(
        step, 0, std::array<double, 2>({{+getParam("length2_bar"), 0}})
    );
    point2contact2->setLine(getParam("p2[0]"), getParam("p2[1]"), p2[0], p2[1]);

}
