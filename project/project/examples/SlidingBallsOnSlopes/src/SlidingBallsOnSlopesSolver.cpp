// SlidingBallsOnSlopesSolver
#include "SlidingBallsOnSlopes/SlidingBallsOnSlopesSolver.hpp"

// std::make_pair
#include <utility>

SlidingBallsOnSlopesSolver::SlidingBallsOnSlopesSolver(

):
SlidingBallsOnSlopesSymb(),
SlidingBallsOnSlopesSolverImpl(
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
    std::string("Sliding balls on slopes")
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