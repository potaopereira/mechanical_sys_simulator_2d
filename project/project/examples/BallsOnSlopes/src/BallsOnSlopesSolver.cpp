// BallsOnSlopesSolver
#include "BallsOnSlopes/BallsOnSlopesSolver.hpp"

// std::make_pair
#include <utility>

BallsOnSlopesSolver::BallsOnSlopesSolver(

):
BallsOnSlopesSymb(),
BallsOnSlopesSolverImpl(
    this  // this is a IMSConstraints<int N, int M1, int M2, int M3> instance
    ,
    std::array<rbi_t, 2>(
        {{
            {0.5, 0.5*0.1*0.1}
            ,
            {0.5, 0.5*0.1*0.1}
        }}
    )
    ,
    std::string("Balls on slopes")
    ,
    std::array<std::string, 2>(
        {{
            std::string("RB1")
            ,
            std::string("RB2")
        }}
    )
)
{

}