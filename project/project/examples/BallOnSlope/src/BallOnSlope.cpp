// BallOnSlopeSymb
#include "BallOnSlope/BallOnSlope.hpp"

BallOnSlopeSymb::BallOnSlopeSymb(

):
BallOnSlopeSymbImpl(
    std::array<sop::tuple_holonomic_constraint_t, 0>({})
    ,
    std::array<sop::tuple_holonomic_constraint_with_contact_t, 1>(
        {
            {
                {
                    .rigid_body = 0,
                    .constraint = &sop::pc_slope,
                    .constraint_parameters = std::vector<Symbolic>({0, 0, Symbolic("slope"), 1}),
                    .boundary = &sop::pc_ellipse,
                    .boundary_parameters = std::vector<Symbolic>({Symbolic("offset1"), Symbolic("offset2"), Symbolic("radius1"), Symbolic("radius2")}),
                    .sliding = false
                }
            }
        }
    )
    ,
    std::string("BallOnSlope")
)
,
IMRB2DPARAM()
{

    slope = 1;
    mParamList["slope"] = std::make_tuple(
        &slope, 0, 1000, "slope", "parameter that controls angle of slope (angle = arctan(slope))"
    );

    offset1 = 0;
    mParamList["offset1"] = std::make_tuple(
        &offset1, -500, 500, "CoG offset 1", "Offset 1 of CoG from geometric center in mm"
    );

    offset2 = 0;
    mParamList["offset2"] = std::make_tuple(
        &offset2, -500, 500, "CoG offset 1", "Offset 2 of CoG from geometric center in mm"
    );

    radius1 = 100;
    mParamList["radius1"] = std::make_tuple(
        &radius1, 1, 1000, "radius 1", "radius 1 of ellipse in mm"
    );

    radius2 = 200;
    mParamList["radius2"] = std::make_tuple(
        &radius2, 1, 1000, "radius 2", "radius 2 of ellipse in mm"
    );
}