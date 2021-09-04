// BarPendulumSymb
#include "BarPendulum/BarPendulum.hpp"

BarPendulumSymb::BarPendulumSymb(

):
BarPendulumSymbImpl(
    std::array<sop::tuple_holonomic_constraint_t, 2>(
        {{
            {
                .constraint = &sop::hc_dist2point,
                .rigid_bodies = std::vector<int>({0}),
                .parameters = std::vector<Symbolic>({Symbolic("p1[0]"), Symbolic("p1[1]"), -Symbolic("length1_bar"), 0, Symbolic("arm_1_squared")}),
            }
            ,
            {
                .constraint = &sop::hc_dist2point,
                .rigid_bodies = std::vector<int>({0}),
                .parameters = std::vector<Symbolic>({Symbolic("p2[0]"), Symbolic("p2[1]"), +Symbolic("length2_bar"), 0, Symbolic("arm_2_squared")}),
            }
        }}
    )
    ,
    std::string("BarPendulum")
)
,
IMRB2DPARAM()
{
    length1_bar = +400;
    mParamList["length1_bar"] = std::make_tuple(
        &length1_bar, 0, +800, "d1", "distance to contact point 1"
    );

    length2_bar = +400;
    mParamList["length2_bar"] = std::make_tuple(
        &length2_bar, 0, +8000, "d2", "distance to contact point 2"
    );

    arm_1_squared = 500*500;
    mParamList["arm_1_squared"] = std::make_tuple(
        &arm_1_squared, 0, 700*700, "l1", "distance squared between contact point 1 and fixed point 1"
    );

    arm_2_squared = +500*500;
    mParamList["arm_2_squared"] = std::make_tuple(
        &arm_2_squared, 0, 700*700, "l2", "distance squared between contact point 2 and fixed point 2"
    );

    p1[0] = -420;
    mParamList["p1[0]"] = std::make_tuple(
        &p1[0], -600, 0, "p1 x", "point 1 x coordinate"
    );

    p1[1] = 500;
    mParamList["p1[1]"] = std::make_tuple(
        &p1[1], -600, 0, "p1 y", "point 1 y coordinate"
    );

    p2[0] = +420;
    mParamList["p2[0]"] = std::make_tuple(
        &p2[0], -600, 0, "p2 x", "point 2 x coordinate"
    );

    p2[1] = 500;
    mParamList["p2[1]"] = std::make_tuple(
        &p2[1], -600, 0, "p2 y", "point 2 y coordinate"
    );
}