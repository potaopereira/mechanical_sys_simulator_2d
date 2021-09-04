// BallOnCircleSolver
#include "BallOnCircle/BallOnCircleSolver.hpp"

// std::make_pair
#include <utility>

BallOnCircleSolver::BallOnCircleSolver(

):
BallOnCircleSymb(),
BallOnCircleSolverImpl(
    this  // this is a IMSConstraints<int N, int M1, int M2, int M3> instance
    ,
    std::array<rbi_t, 1>(
        {{
            {
                1, // mass
                1*(getParam("ball_a0")*getParam("ball_a0") + getParam("ball_a1")*getParam("ball_a1"))/2, // mass * radius^2
            }
        }}
    )
    ,
    std::string("Ball on Circle")
    ,
    std::array<std::string, 1>(
        {{
            std::string("RB1")
        }}
    )
)
{

}