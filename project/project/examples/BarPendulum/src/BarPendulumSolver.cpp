// BarPendulumSolver
#include "BarPendulum/BarPendulumSolver.hpp"

// std::make_pair
#include <utility>

BarPendulumSolver::BarPendulumSolver(

):
BarPendulumSymb(),
BarPendulumSolverImpl(
    std::array<rbi_t, 1>(
        {{
            {0.5, 0.5*0.1*0.1}
        }}
    )
    ,
    std::string("Bar Pendulum")
    ,
    std::array<std::string, 1>(
        {{
            std::string("RB1")
        }}
    )
)
{

}