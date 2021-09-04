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
                .parameters = std::vector<Symbolic>({-100, +150, -Symbolic("length1_bar"), 0, Symbolic("arm_1_squared")}),
            }
            ,
            {
                .constraint = &sop::hc_dist2point,
                .rigid_bodies = std::vector<int>({0}),
                .parameters = std::vector<Symbolic>({+100, +150, +Symbolic("length2_bar"), 0, Symbolic("arm_2_squared")}),
            }
        }}
    )
    ,
    std::string("BarPendulum")
)
,
IMRB2DPARAM()
{
    length1_bar = +100;
    mParamList["length1_bar"] = std::make_tuple(
        &length1_bar, 0, +200, "d1", "distance to contact point 1"
    );

    length2_bar = +100;
    mParamList["length2_bar"] = std::make_tuple(
        &length2_bar, 0, +200, "d2", "distance to contact point 2"
    );

    arm_1_squared = 150*150;
    mParamList["arm_1_squared"] = std::make_tuple(
        &arm_1_squared, 0, 170*170, "l1", "distance between contact point 1 and fixed point 1"
    );

    arm_2_squared = +150*150;
    mParamList["arm_2_squared"] = std::make_tuple(
        &arm_2_squared, 0, 170*170, "l2", "distance between contact point 2 and fixed point 2"
    );
}