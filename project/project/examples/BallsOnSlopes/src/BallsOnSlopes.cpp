// BallsOnSlopesSymb
#include "BallsOnSlopes/BallsOnSlopes.hpp"

BallsOnSlopesSymb::BallsOnSlopesSymb(

):
BallsOnSlopesSymbImpl(
    std::array<sop::tuple_holonomic_constraint_t, 0>({})
    ,
    std::array<sop::tuple_holonomic_constraint_with_contact_t, 2>(
        {
            {
                {
                    .rigid_body = 0,
                    .constraint = &sop::pc_slope,
                    .constraint_parameters = std::vector<Symbolic>({0, 0, Symbolic("slope1"), 1}),
                    .boundary = &sop::pc_ellipse,
                    .boundary_parameters = std::vector<Symbolic>({0, 0, Symbolic("radius1_1"), Symbolic("radius1_2")}),
                    .sliding = false
                }
                ,
                {
                    .rigid_body = 1,
                    .constraint = &sop::pc_slope,
                    .constraint_parameters = std::vector<Symbolic>({0, 0, 1, Symbolic("slope2")}),
                    .boundary = &sop::pc_ellipse,
                    .boundary_parameters = std::vector<Symbolic>({0, 0, Symbolic("radius2_1"), Symbolic("radius2_2")}),
                    .sliding = false
                }
            }
        }
    )
    ,
    std::string("BallsOnSlopes")
)
,
IMRB2DPARAM()
{

    slope_1 = 1;
    mParamList["slope_1"] = std::make_tuple(
        &slope, 0, 1000, "slope 1", "parameter that controls angle of slope 1 (angle = arctan(slope 1))"
    );

    radius1_1 = 100;
    mParamList["radius1_1"] = std::make_tuple(
        &radius1, 1, 1000, "radius 1 of ball 1", "radius 1 of ellipse 1 in mm"
    );

    radius1_2 = 200;
    mParamList["radius1_2"] = std::make_tuple(
        &radius2, 1, 1000, "radius 2 of ball 1", "radius 2 of ellipse 1 in mm"
    );

    slope_2 = 1;
    mParamList["slope_2"] = std::make_tuple(
        &slope, 0, 1000, "slope 2", "parameter that controls angle of slope 2 (angle = pi - arctan(slope 2))"
    );

    radius2_1 = 100;
    mParamList["radius2_1"] = std::make_tuple(
        &radius2_1, 1, 1000, "radius 1 of ball 2", "radius 1 of ellipse 2 in mm"
    );

    radius2_2 = 200;
    mParamList["radius2_2"] = std::make_tuple(
        &radius2_2, 1, 1000, "radius 2 of ball 2", "radius 2 of ellipse 2 in mm"
    );
}