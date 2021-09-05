#include "SlidingBallsOnSlopes/SlidingBallsOnSlopesViewer.hpp"

#include <msviewer/rbwithboundary.hpp>


SlidingBallsOnSlopesViewer::SlidingBallsOnSlopesViewer(

):
SlidingBallsOnSlopesSolver(),
MSVIEW2(
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
    p1Top2 = new QGraphicsLineItem();
    p1Top2->setLine(0, 0, 0, 0);

    slope_1 = new QGraphicsLineItem();
    slope_1->setLine(0 + 1000*(-1), 0 + 1000*(+getParam("slope_1")), 0 + 1000*(+1), 0 + 1000*(-getParam("slope_1")));

    slope_2 = new QGraphicsLineItem();
    slope_2->setLine(0 + 1000*(-getParam("slope_2")), 0 + 1000*(+1), 0 + 1000*(+getParam("slope_2")), 0 + 1000*(-1));

    std::vector<QGraphicsItem*> listOfConstraints = {slope_1, slope_2, p1Top2};

    // MSVIEW2::setConstraints
    setConstraints(listOfConstraints);

    p0 <<
    -sqrt(getParam("l_squared")) - getParam("radius2_1"), getParam("radius1_2"),
    1,0,0,1,
    - getParam("radius2_1"), getParam("radius2_2"),
    1,0,0,1;
    
    q0 <<
    0, -getParam("radius1_2"),
    getParam("radius2_1"), 0;
    
    v0 <<
    0, 0, 0,
    0, 0, 0;

}

void
SlidingBallsOnSlopesViewer::reset(){

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

    // MSVIEW2
    MSVIEW2::reset(
        std::array<void*, 2>(
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

void
SlidingBallsOnSlopesViewer::showExtraAtStep(
    int step
){

    std::array<double, 2> p1 = IMS2DSolver::getPositionOfPoint(
        step, 0, std::array<double, 2>({{0, 0}})
    );
    std::array<double, 2> p2 = IMS2DSolver::getPositionOfPoint(
        step, 1, std::array<double, 2>({{0, 0}})
    );
    p1Top2->setLine(p1[0], p1[1], p2[0], p2[1]);

}
