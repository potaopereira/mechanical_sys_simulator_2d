// BallOnCircleSymb
#include "BallOnCircle/BallOnCircle.hpp"

BallOnCircleSymb::BallOnCircleSymb(

):
BallOnCircleSymbImpl(
    std::array<sop::tuple_holonomic_constraint_t, 0>({})
    ,
    std::array<sop::tuple_holonomic_constraint_with_contact_t, 1>(
        {
            {
                {
                    .rigid_body = 0,
                    .constraint = &sop::pc_ellipse,
                    .constraint_parameters = std::vector<Symbolic>({0, 0, Symbolic("outside_a0"), Symbolic("outside_a1")}),
                    .boundary = &sop::pc_ellipse,
                    .boundary_parameters = std::vector<Symbolic>(
                            {
                                Symbolic("ball_offset0"),
                                Symbolic("ball_offset1"),
                                Symbolic("ball_a0"),
                                Symbolic("ball_a1")
                            }
                        ),
                    .sliding = false
                }
            }
        }
    )
    ,
    std::string("BallOnCircle")
)
,
IMRB2DPARAM()
{
    outside_a0 = 900;
    mParamList["outside_a0"] = std::make_tuple(
        &outside_a0, 1, 1000, "outside radius 1", "contact ellipse axis 1 length mm"
    );
    outside_a1 = 900;
    mParamList["outside_a1"] = std::make_tuple(
        &outside_a1, 1, 1000, "outside radius 2", "contact ellipse axis 2 length mm"
    );

    ball_a0 = 100;
    mParamList["ball_a0"] = std::make_tuple(
        &ball_a0, 1, 1000, "ball radius 1", "ball radius axis 1 length mm"
    );
    ball_a1 = 200;
    mParamList["ball_a1"] = std::make_tuple(
        &ball_a1, 1, 1000, "ball radius 2", "ball radius axis 2 length mm"
    );

    ball_offset0 = 0;
    mParamList["ball_offset0"] = std::make_tuple(
        &ball_offset0, 1, 1000, "CoG offset 1", "Offset in center of gravity in mm in direction 1"
    );
    ball_offset1 = 0;
    mParamList["ball_offset1"] = std::make_tuple(
        &ball_offset1, 1, 1000, "CoG offset 2", "Offset in center of gravity in mm in direction 2"
    );

}