#include "examplems/examplems.hpp"

ExampleMSSymb::ExampleMSSymb(

):
ExampleMSSymbImpl(
    std::array<sop::tuple_holonomic_constraint_t, 0>({})
    ,
    std::array<sop::tuple_holonomic_constraint_with_contact_t, 1>(
        {
            {
                {
                    .rigid_body = 0,
                    .constraint = &sop::pc_slope,
                    .constraint_parameters = std::vector<Symbolic>({0, 0, 1, 1}),
                    .boundary = &sop::pc_ellipse,
                    .boundary_parameters = std::vector<Symbolic>({0, 0, Symbolic("parameter1"), Symbolic("parameter2")}),
                    .sliding = false
                }
            }
        }
    )
    ,
    std::string("ExampleMS")
)
,
IMRB2DPARAM()
{
    parameter1 = 100;
    mParamList["parameter1"] = std::make_tuple(
        &parameter1, 1, 1000, "parameter1", "parameter1 description"
    );

    parameter2 = 200;
    mParamList["parameter2"] = std::make_tuple(
        &parameter2, 1, 1000, "parameter2", "parameter2 description"
    );
}