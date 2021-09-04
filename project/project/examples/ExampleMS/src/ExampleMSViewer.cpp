#include "ExampleMS/ExampleMSViewer.hpp"

#include <msviewer/rbwithboundary.hpp>


ExampleMSViewer::ExampleMSViewer(

):
ExampleMSSolver(),
MSVIEW1(
    std::array<RBViewWVectors*, 1>(
        {
            {
                // rigid body 0
                new RBViewEllipse(
                        Eigen::Matrix<double, 2, 1>(0, 0),
                        Eigen::Matrix<double, 2, 1>(getParam("parameter1"), getParam("parameter2")),
                        0
                    )
            }
        }
    )
),
IMS2D(this, this, this)
{

    QGraphicsLineItem* c1 = new QGraphicsLineItem();
    c1->setLine(-1000, 1000, 1000, -1000);
    std::vector<QGraphicsItem*> listOfConstraints = {c1};

    // MSVIEW1::setConstraints
    setConstraints(listOfConstraints);

    p0 <<  getParam("parameter1")*cos(M_PI/4), getParam("parameter2")*sin(M_PI/4), cos(M_PI/4), -sin(M_PI/4), sin(M_PI/4), cos(M_PI/4);
    q0 << -getParam("parameter1"), 0;
    // v0 << 0.5, -0.5, 1;
    v0 << 0, 0, 0;

}

void
ExampleMSViewer::reset(){

    // rigid body 0
    RBViewEllipse::param_t param1 =
        std::make_tuple(
            Eigen::Matrix<double, 2, 1>(0, 0),
            Eigen::Matrix<double, 2, 1>(getParam("parameter1"), getParam("parameter2")),
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