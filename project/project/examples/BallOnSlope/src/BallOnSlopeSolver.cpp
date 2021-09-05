// BallOnSlopeSolver
#include "BallOnSlope/BallOnSlopeSolver.hpp"

// std::make_pair
#include <utility>

BallOnSlopeSolver::BallOnSlopeSolver(

):
BallOnSlopeSymb(),
BallOnSlopeSolverImpl(
    this  // this is a IMSConstraints<int N, int M1, int M2, int M3> instance
    ,
    std::array<rbi_t, 1>(
        {{
            {0.5, 0.5*0.1*0.1}
        }}
    )
    ,
    std::string("Ball on slope")
    ,
    std::array<std::string, 1>(
        {{
            std::string("RB1")
        }}
    )
)
{

}