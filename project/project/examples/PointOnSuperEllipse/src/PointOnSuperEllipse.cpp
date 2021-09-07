// PointOnSuperEllipseSymb
#include "PointOnSuperEllipse/PointOnSuperEllipse.hpp"

PointOnSuperEllipseSymb::PointOnSuperEllipseSymb(

):
PointOnSuperEllipseSymbImpl(
    std::array<sop::tuple_holonomic_constraint_t, 1>(
        {{
            {
                .constraint = &sop::hc_superellipse,
                .rigid_bodies = std::vector<int>({0}),
                .parameters = std::vector<Symbolic>({0, 0, Symbolic("outside_rinv"), Symbolic("outside_rinv"), Symbolic("outside_alpha")})
            }
        }}
    )
    ,
    std::string("PointOnSuperEllipse")
)
,
IMRB2DPARAM()
{
    outside_rinv = 1./900;
    mParamList["outside_rinv"] = std::make_tuple(
        &outside_rinv, 1./100, 1./1000, "outside curvature", "contact super-ellipse reciprocal of radius in 1 / mm"
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