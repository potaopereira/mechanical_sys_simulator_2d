// ExampleMSSolver
#include "ExampleMS/ExampleMSSolver.hpp"

// std::make_pair
#include <utility>

ExampleMSSolver::ExampleMSSolver(

):
ExampleMSSymb(),
ExampleMSSolverImpl(
    this  // this is a IMSConstraints<int N, int M1, int M2, int M3> instance
    ,
    std::array<rbi_t, 1>(
        {{
            {0.5, 0.5*0.1*0.1}
        }}
    )
    ,
    std::string("Example")
    ,
    std::array<std::string, 1>(
        {{
            std::string("RB1")
        }}
    )
)
{

}