#include "BallsOnSlopes/BallsOnSlopesViewer.hpp"

#include <msviewer/rbwithboundary.hpp>


BallsOnSlopesViewer::BallsOnSlopesViewer(

):
BallsOnSlopesSolver(),
MSVIEW1(
    std::array<RBViewWVectors*, 2>(
        {{
            // rigid body 0
            new RBViewEllipse(
                    Eigen::Matrix<double, 2, 1>(0, 0),
                    Eigen::Matrix<double, 2, 1>(getParam("radius1_1"), getParam("radius1_2")),
                    0
                )
            ,
            // rigid body 1
            new RBViewEllipse(
                    Eigen::Matrix<double, 2, 1>(0, 0),
                    Eigen::Matrix<double, 2, 1>(getParam("radius2_1"), getParam("radius2_2")),
                    0
                )
        }}
    )
),
IMS2D(this, this, this)
{

    slope_1 = new QGraphicsLineItem();
    slope_1->setLine(0 + 1000*(-1), 0 + 1000*(+getParam("slope_1")), 0 + 1000*(+1), 0 + 1000*(-getParam("slope_1")));

    slope_2 = new QGraphicsLineItem();
    slope_2->setLine(0 + 1000*(-getParam("slope_2")), 0 + 1000*(+1), 0 + 1000*(+getParam("slope_2")), 0 + 1000*(-1));

    std::vector<QGraphicsItem*> listOfConstraints = {slope_1, slope_2};

    // MSVIEW1::setConstraints
    setConstraints(listOfConstraints);

    p0 << 
    ,,
    1,0,0,1,
    ,,
    1,0,0,1;
    
    q0 <<
    getParam("radius1_1"), 0,
    getParam("radius2_1"), 0;
    
    v0 <<
    0, 0, 0,
    0, 0, 0;

}

void
BallsOnSlopesViewer::reset(){

    // rigid body 0
    RBViewEllipse::param_t param0 =
        std::make_tuple(
            Eigen::Matrix<double, 2, 1>(0, 0),
            Eigen::Matrix<double, 2, 1>(getParam("radius1_1"), getParam("radius1_2")),
            0
        );

    // rigid body 1
    RBViewEllipse::param_t param1 =
        std::make_tuple(
            Eigen::Matrix<double, 2, 1>(0, 0),
            Eigen::Matrix<double, 2, 1>(getParam("radius2_1"), getParam("radius2_2")),
            0
        );

    // MSVIEW1
    MSVIEW1::reset(
        std::array<void*, 1>(
            {{
                &param0
                ,
                &param1
            }}
        )
    );

    slope_1->setLine(0 + 1000*(-1), 0 + 1000*(+getParam("slope_1")), 0 + 1000*(+1), 0 + 1000*(-getParam("slope_1")));
    slope_2->setLine(0 + 1000*(-getParam("slope_2")), 0 + 1000*(+1), 0 + 1000*(+getParam("slope_2")), 0 + 1000*(-1));

}