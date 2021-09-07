// BallOnSuperEllipseSolver
#include "BallOnSuperEllipse/BallOnSuperEllipseSolver.hpp"

// std::make_pair
#include <utility>

BallOnSuperEllipseSolver::BallOnSuperEllipseSolver(

):
BallOnSuperEllipseSymb(),
BallOnSuperEllipseSolverImpl(
    std::array<rbi_t, 1>(
        {{
            {
                1, // mass
                1, // mass * radius^2
            }
        }}
    )
    ,
    std::string("Ball on Super-ellipse")
    ,
    std::array<std::string, 1>(
        {{
            std::string("RB1")
        }}
    )
)
{

}