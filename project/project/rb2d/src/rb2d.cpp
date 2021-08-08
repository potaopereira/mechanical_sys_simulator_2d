#include "rb2d/rb2d.hpp"

// cos, sin
#include <math.h>

LinearPosition::LinearPosition(
    double const p1
    ,
    double const p2
)
:
p{p1,p2}
{

}

AngularPosition::AngularPosition(
    double theta
):
p{cos(theta), -sin(theta), +sin(theta), cos(theta)}
{

}

Position::Position(

)
{

}

RB2D::RB2D(

)
{

}