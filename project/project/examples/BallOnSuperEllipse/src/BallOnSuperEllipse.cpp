// BallOnSuperEllipseSymb
#include "BallOnSuperEllipse/BallOnSuperEllipse.hpp"

BallOnSuperEllipseSymb::BallOnSuperEllipseSymb(

):
BallOnSuperEllipseSymbImpl(
    std::array<sop::tuple_holonomic_constraint_t, 0>({})
    ,
    std::array<sop::tuple_holonomic_constraint_with_contact_t, 1>(
        {{
                {
                    .rigid_body = 0,
                    .constraint = &sop::pc_superellipse,
                    .constraint_parameters = std::vector<Symbolic>({0, 0, Symbolic("outside_a"), Symbolic("outside_a"), Symbolic("outside_alpha")}),
                    .boundary = &sop::pc_ellipse,
                    .boundary_parameters = std::vector<Symbolic>({0, 0, Symbolic("ball_r"), Symbolic("ball_r")}),
                    .sliding = false
                }
        }}
    )
    ,
    std::string("BallOnSuperEllipse")
)
,
IMRB2DPARAM()
{
    outside_a = 1./900;
    mParamList["outside_a"] = std::make_tuple(
        &outside_a, 1./100, 1./1000, "outside curvature", "contact super-ellipse reciprocal of radius in 1 / mm"
    );
    outside_alpha = 2;
    mParamList["outside_alpha"] = std::make_tuple(
        &outside_alpha, 1, 10, "outside alpha", "contact super-ellipse alpha factor"
    );

    ball_r = 5;
    mParamList["ball_r"] = std::make_tuple(
        &ball_r, 1, 10, "ball radius", "ball radius"
    );

}