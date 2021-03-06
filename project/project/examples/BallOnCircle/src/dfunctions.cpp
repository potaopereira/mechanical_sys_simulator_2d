// BallOnCircleSolver
#include "BallOnCircle/BallOnCircleSolver.hpp"

BallOnCircleSolver::c_t
BallOnCircleSolver::get_c(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double q00 = q(0);
    double q01 = q(1);
    c_t out;
    out << 
        pow(p00,2)*pow(outside_a0,-2)+2*p00*p02*q00*pow(outside_a0,-2)+2*p00*p03*q01*pow(outside_a0,-2)+pow(p02,2)*pow(q00,2)*pow(outside_a0,-2)+2*p02*q00*p03*q01*pow(outside_a0,-2)+pow(p03,2)*pow(q01,2)*pow(outside_a0,-2)+pow(p01,2)*pow(outside_a1,-2)+2*p01*p04*q00*pow(outside_a1,-2)+2*p01*p05*q01*pow(outside_a1,-2)+pow(p04,2)*pow(q00,2)*pow(outside_a1,-2)+2*p04*q00*p05*q01*pow(outside_a1,-2)+pow(p05,2)*pow(q01,2)*pow(outside_a1,-2)-1,
        pow(q00,2)*pow(ball_a0,-2)-2*q00*ball_offset0*pow(ball_a0,-2)+pow(ball_offset0,2)*pow(ball_a0,-2)+pow(q01,2)*pow(ball_a1,-2)-2*q01*ball_offset1*pow(ball_a1,-2)+pow(ball_offset1,2)*pow(ball_a1,-2)-1,
        4*p01*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*p01*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*p01*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*p01*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)+4*p04*pow(q00,2)*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*p04*q00*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*p04*q00*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*p04*q00*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)+4*p05*q01*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*p05*q01*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*p05*pow(q01,2)*pow(outside_a1,-2)*p03*pow(ball_a1,-2)-4*p05*q01*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*p00*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*p00*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*p00*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)-4*p02*pow(q00,2)*pow(outside_a0,-2)*p04*pow(ball_a0,-2)+4*p02*q00*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*p02*q00*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*p02*q00*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)-4*p03*q01*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*p03*q01*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*p03*pow(q01,2)*pow(outside_a0,-2)*p05*pow(ball_a1,-2)+4*p03*q01*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2),
        0;
    return out;
}

BallOnCircleSolver::cextra_t
BallOnCircleSolver::get_cextra(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double q00 = q(0);
    double q01 = q(1);
    cextra_t out;
    out << 
        pow(p00,2)*pow(outside_a0,-2)+2*p00*p02*q00*pow(outside_a0,-2)+2*p00*p03*q01*pow(outside_a0,-2)+pow(p02,2)*pow(q00,2)*pow(outside_a0,-2)+2*p02*q00*p03*q01*pow(outside_a0,-2)+pow(p03,2)*pow(q01,2)*pow(outside_a0,-2)+pow(p01,2)*pow(outside_a1,-2)+2*p01*p04*q00*pow(outside_a1,-2)+2*p01*p05*q01*pow(outside_a1,-2)+pow(p04,2)*pow(q00,2)*pow(outside_a1,-2)+2*p04*q00*p05*q01*pow(outside_a1,-2)+pow(p05,2)*pow(q01,2)*pow(outside_a1,-2)-1,
        pow(q00,2)*pow(ball_a0,-2)-2*q00*ball_offset0*pow(ball_a0,-2)+pow(ball_offset0,2)*pow(ball_a0,-2)+pow(q01,2)*pow(ball_a1,-2)-2*q01*ball_offset1*pow(ball_a1,-2)+pow(ball_offset1,2)*pow(ball_a1,-2)-1,
        4*p01*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*p01*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*p01*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*p01*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)+4*p04*pow(q00,2)*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*p04*q00*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*p04*q00*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*p04*q00*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)+4*p05*q01*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*p05*q01*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*p05*pow(q01,2)*pow(outside_a1,-2)*p03*pow(ball_a1,-2)-4*p05*q01*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*p00*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*p00*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*p00*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)-4*p02*pow(q00,2)*pow(outside_a0,-2)*p04*pow(ball_a0,-2)+4*p02*q00*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*p02*q00*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*p02*q00*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)-4*p03*q01*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*p03*q01*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*p03*pow(q01,2)*pow(outside_a0,-2)*p05*pow(ball_a1,-2)+4*p03*q01*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2),
        pow(p02,2)+pow(p03,2)-1,
        pow(p04,2)+pow(p05,2)-1,
        p02*p04+p03*p05;
    return out;
}

BallOnCircleSolver::d1cextra_t
BallOnCircleSolver::get_d1cextra(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double q00 = q(0);
    double q01 = q(1);
    d1cextra_t out;
    out << 
        2*p00*pow(outside_a0,-2)+2*p02*q00*pow(outside_a0,-2)+2*p03*q01*pow(outside_a0,-2), 2*p01*pow(outside_a1,-2)+2*p04*q00*pow(outside_a1,-2)+2*p05*q01*pow(outside_a1,-2), 2*p00*q00*pow(outside_a0,-2)+2*p02*pow(q00,2)*pow(outside_a0,-2)+2*q00*p03*q01*pow(outside_a0,-2), 2*p00*q01*pow(outside_a0,-2)+2*p02*q00*q01*pow(outside_a0,-2)+2*p03*pow(q01,2)*pow(outside_a0,-2), 2*p01*q00*pow(outside_a1,-2)+2*p04*pow(q00,2)*pow(outside_a1,-2)+2*q00*p05*q01*pow(outside_a1,-2), 2*p01*q01*pow(outside_a1,-2)+2*p04*q00*q01*pow(outside_a1,-2)+2*p05*pow(q01,2)*pow(outside_a1,-2), 
        0, 0, 0, 0, 0, 0, 
        -4*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2), 4*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2), 4*p01*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p01*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)+4*p04*pow(q00,2)*pow(outside_a1,-2)*pow(ball_a0,-2)-4*p04*q00*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)+4*p05*q01*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p05*q01*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)-4*pow(q00,2)*pow(outside_a0,-2)*p04*pow(ball_a0,-2)+4*q00*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*q00*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*q00*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2), 4*p01*pow(outside_a1,-2)*q01*pow(ball_a1,-2)-4*p01*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)+4*p04*q00*pow(outside_a1,-2)*q01*pow(ball_a1,-2)-4*p04*q00*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)+4*p05*pow(q01,2)*pow(outside_a1,-2)*pow(ball_a1,-2)-4*p05*q01*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)-4*q01*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*q01*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*pow(q01,2)*pow(outside_a0,-2)*p05*pow(ball_a1,-2)+4*q01*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2), 4*pow(q00,2)*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*q00*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*q00*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*q00*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p00*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p02*pow(q00,2)*pow(outside_a0,-2)*pow(ball_a0,-2)+4*p02*q00*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p03*q01*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p03*q01*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2), 4*q01*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*q01*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*pow(q01,2)*pow(outside_a1,-2)*p03*pow(ball_a1,-2)-4*q01*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*q01*pow(ball_a1,-2)+4*p00*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2)-4*p02*q00*pow(outside_a0,-2)*q01*pow(ball_a1,-2)+4*p02*q00*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2)-4*p03*pow(q01,2)*pow(outside_a0,-2)*pow(ball_a1,-2)+4*p03*q01*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2), 
        0, 0, 2*p02, 2*p03, 0, 0, 
        0, 0, 0, 0, 2*p04, 2*p05, 
        0, 0, p04, p05, p02, p03;
    return out;
}

BallOnCircleSolver::d2cextra_t
BallOnCircleSolver::get_d2cextra(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double q00 = q(0);
    double q01 = q(1);
    d2cextra_t out;
    out << 
        2*p00*p02*pow(outside_a0,-2)+2*pow(p02,2)*q00*pow(outside_a0,-2)+2*p02*p03*q01*pow(outside_a0,-2)+2*p01*p04*pow(outside_a1,-2)+2*pow(p04,2)*q00*pow(outside_a1,-2)+2*p04*p05*q01*pow(outside_a1,-2), 2*p00*p03*pow(outside_a0,-2)+2*p02*q00*p03*pow(outside_a0,-2)+2*pow(p03,2)*q01*pow(outside_a0,-2)+2*p01*p05*pow(outside_a1,-2)+2*p04*q00*p05*pow(outside_a1,-2)+2*pow(p05,2)*q01*pow(outside_a1,-2), 
        2*q00*pow(ball_a0,-2)-2*ball_offset0*pow(ball_a0,-2), 2*q01*pow(ball_a1,-2)-2*ball_offset1*pow(ball_a1,-2), 
        4*p01*pow(outside_a1,-2)*p02*pow(ball_a0,-2)+8*p04*q00*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*p04*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*p04*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*p04*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)+4*p05*q01*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*p00*pow(outside_a0,-2)*p04*pow(ball_a0,-2)-8*p02*q00*pow(outside_a0,-2)*p04*pow(ball_a0,-2)+4*p02*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*p02*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*p02*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)-4*p03*q01*pow(outside_a0,-2)*p04*pow(ball_a0,-2), 4*p01*pow(outside_a1,-2)*p03*pow(ball_a1,-2)+4*p04*q00*pow(outside_a1,-2)*p03*pow(ball_a1,-2)+4*p05*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*p05*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+8*p05*q01*pow(outside_a1,-2)*p03*pow(ball_a1,-2)-4*p05*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*p05*pow(ball_a1,-2)-4*p02*q00*pow(outside_a0,-2)*p05*pow(ball_a1,-2)-4*p03*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*p03*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-8*p03*q01*pow(outside_a0,-2)*p05*pow(ball_a1,-2)+4*p03*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2), 
        0, 0, 
        0, 0, 
        0, 0;
    return out;
}

BallOnCircleSolver::d1ck_t
BallOnCircleSolver::get_d1ck(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double q00 = q(0);
    double q01 = q(1);
    d1ck_t out;
    out <<
        2*p00*pow(outside_a0,-2)+2*p02*q00*pow(outside_a0,-2)+2*p03*q01*pow(outside_a0,-2), 2*p01*pow(outside_a1,-2)+2*p04*q00*pow(outside_a1,-2)+2*p05*q01*pow(outside_a1,-2), 2*p03*p00*q00*pow(outside_a0,-2)+2*p03*p02*pow(q00,2)*pow(outside_a0,-2)+2*pow(p03,2)*q00*q01*pow(outside_a0,-2)-2*p02*p00*q01*pow(outside_a0,-2)-2*pow(p02,2)*q00*q01*pow(outside_a0,-2)-2*p02*p03*pow(q01,2)*pow(outside_a0,-2)+2*p05*p01*q00*pow(outside_a1,-2)+2*p05*p04*pow(q00,2)*pow(outside_a1,-2)+2*pow(p05,2)*q00*q01*pow(outside_a1,-2)-2*p04*p01*q01*pow(outside_a1,-2)-2*pow(p04,2)*q00*q01*pow(outside_a1,-2)-2*p04*p05*pow(q01,2)*pow(outside_a1,-2), 
        0, 0, 0, 
        -4*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2), 4*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2), 4*p03*p01*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p03*p01*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)+4*p03*p04*pow(q00,2)*pow(outside_a1,-2)*pow(ball_a0,-2)-4*p03*p04*q00*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)+4*p03*p05*q01*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p03*p05*q01*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)-4*p03*pow(q00,2)*pow(outside_a0,-2)*p04*pow(ball_a0,-2)+4*p03*q00*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*p03*q00*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*p03*q00*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)-4*p02*p01*pow(outside_a1,-2)*q01*pow(ball_a1,-2)+4*p02*p01*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)-4*p02*p04*q00*pow(outside_a1,-2)*q01*pow(ball_a1,-2)+4*p02*p04*q00*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)-4*p02*p05*pow(q01,2)*pow(outside_a1,-2)*pow(ball_a1,-2)+4*p02*p05*q01*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)+4*p02*q01*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)-4*p02*q01*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)+4*p02*pow(q01,2)*pow(outside_a0,-2)*p05*pow(ball_a1,-2)-4*p02*q01*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)+4*p05*pow(q00,2)*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*p05*q00*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*p05*q00*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*p05*q00*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p05*p00*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p05*p00*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p05*p02*pow(q00,2)*pow(outside_a0,-2)*pow(ball_a0,-2)+4*p05*p02*q00*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p05*p03*q01*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p05*p03*q01*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p04*q01*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)+4*p04*q01*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)-4*p04*pow(q01,2)*pow(outside_a1,-2)*p03*pow(ball_a1,-2)+4*p04*q01*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)+4*p04*p00*pow(outside_a0,-2)*q01*pow(ball_a1,-2)-4*p04*p00*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2)+4*p04*p02*q00*pow(outside_a0,-2)*q01*pow(ball_a1,-2)-4*p04*p02*q00*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2)+4*p04*p03*pow(q01,2)*pow(outside_a0,-2)*pow(ball_a1,-2)-4*p04*p03*q01*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2), 
        2*p01*pow(outside_a1,-2)+2*p04*q00*pow(outside_a1,-2)+2*p05*q01*pow(outside_a1,-2), -2*p00*pow(outside_a0,-2)-2*p02*q00*pow(outside_a0,-2)-2*p03*q01*pow(outside_a0,-2), -2*p01*pow(outside_a1,-2)*p02*q01+2*p01*pow(outside_a1,-2)*p03*q00-2*p04*q00*pow(outside_a1,-2)*p02*q01+2*p04*pow(q00,2)*pow(outside_a1,-2)*p03-2*p05*pow(q01,2)*pow(outside_a1,-2)*p02+2*p05*q01*pow(outside_a1,-2)*p03*q00+2*p00*pow(outside_a0,-2)*p04*q01-2*p00*pow(outside_a0,-2)*p05*q00+2*p02*q00*pow(outside_a0,-2)*p04*q01-2*p02*pow(q00,2)*pow(outside_a0,-2)*p05+2*p03*pow(q01,2)*pow(outside_a0,-2)*p04-2*p03*q01*pow(outside_a0,-2)*p05*q00;
    return out;
}

BallOnCircleSolver::d1c_t
BallOnCircleSolver::get_d1c(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double q00 = q(0);
    double q01 = q(1);
    d1c_t out;
    out <<
        2*p00*pow(outside_a0,-2)+2*p02*q00*pow(outside_a0,-2)+2*p03*q01*pow(outside_a0,-2), 2*p01*pow(outside_a1,-2)+2*p04*q00*pow(outside_a1,-2)+2*p05*q01*pow(outside_a1,-2), 2*p00*q00*pow(outside_a0,-2)+2*p02*pow(q00,2)*pow(outside_a0,-2)+2*q00*p03*q01*pow(outside_a0,-2), 2*p00*q01*pow(outside_a0,-2)+2*p02*q00*q01*pow(outside_a0,-2)+2*p03*pow(q01,2)*pow(outside_a0,-2), 2*p01*q00*pow(outside_a1,-2)+2*p04*pow(q00,2)*pow(outside_a1,-2)+2*q00*p05*q01*pow(outside_a1,-2), 2*p01*q01*pow(outside_a1,-2)+2*p04*q00*q01*pow(outside_a1,-2)+2*p05*pow(q01,2)*pow(outside_a1,-2), 
        0, 0, 0, 0, 0, 0, 
        -4*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2), 4*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2), 4*p01*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p01*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)+4*p04*pow(q00,2)*pow(outside_a1,-2)*pow(ball_a0,-2)-4*p04*q00*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)+4*p05*q01*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p05*q01*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)-4*pow(q00,2)*pow(outside_a0,-2)*p04*pow(ball_a0,-2)+4*q00*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*q00*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*q00*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2), 4*p01*pow(outside_a1,-2)*q01*pow(ball_a1,-2)-4*p01*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)+4*p04*q00*pow(outside_a1,-2)*q01*pow(ball_a1,-2)-4*p04*q00*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)+4*p05*pow(q01,2)*pow(outside_a1,-2)*pow(ball_a1,-2)-4*p05*q01*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)-4*q01*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*q01*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*pow(q01,2)*pow(outside_a0,-2)*p05*pow(ball_a1,-2)+4*q01*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2), 4*pow(q00,2)*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*q00*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*q00*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*q00*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p00*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p02*pow(q00,2)*pow(outside_a0,-2)*pow(ball_a0,-2)+4*p02*q00*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p03*q01*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p03*q01*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2), 4*q01*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*q01*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*pow(q01,2)*pow(outside_a1,-2)*p03*pow(ball_a1,-2)-4*q01*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*q01*pow(ball_a1,-2)+4*p00*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2)-4*p02*q00*pow(outside_a0,-2)*q01*pow(ball_a1,-2)+4*p02*q00*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2)-4*p03*pow(q01,2)*pow(outside_a0,-2)*pow(ball_a1,-2)+4*p03*q01*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2), 
        2*p01*pow(outside_a1,-2)+2*p04*q00*pow(outside_a1,-2)+2*p05*q01*pow(outside_a1,-2), -2*p00*pow(outside_a0,-2)-2*p02*q00*pow(outside_a0,-2)-2*p03*q01*pow(outside_a0,-2), 2*p01*pow(outside_a1,-2)*q00+2*p04*pow(q00,2)*pow(outside_a1,-2)+2*p05*q01*pow(outside_a1,-2)*q00, 2*p01*pow(outside_a1,-2)*q01+2*p04*q00*pow(outside_a1,-2)*q01+2*p05*pow(q01,2)*pow(outside_a1,-2), -2*p00*pow(outside_a0,-2)*q00-2*p02*pow(q00,2)*pow(outside_a0,-2)-2*p03*q01*pow(outside_a0,-2)*q00, -2*p00*pow(outside_a0,-2)*q01-2*p02*q00*pow(outside_a0,-2)*q01-2*p03*pow(q01,2)*pow(outside_a0,-2);
    return out;
}

BallOnCircleSolver::d2c_t
BallOnCircleSolver::get_d2c(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double q00 = q(0);
    double q01 = q(1);
    d2c_t out;
    out <<
        2*p00*p02*pow(outside_a0,-2)+2*pow(p02,2)*q00*pow(outside_a0,-2)+2*p02*p03*q01*pow(outside_a0,-2)+2*p01*p04*pow(outside_a1,-2)+2*pow(p04,2)*q00*pow(outside_a1,-2)+2*p04*p05*q01*pow(outside_a1,-2),2*p00*p03*pow(outside_a0,-2)+2*p02*q00*p03*pow(outside_a0,-2)+2*pow(p03,2)*q01*pow(outside_a0,-2)+2*p01*p05*pow(outside_a1,-2)+2*p04*q00*p05*pow(outside_a1,-2)+2*pow(p05,2)*q01*pow(outside_a1,-2),
        2*q00*pow(ball_a0,-2)-2*ball_offset0*pow(ball_a0,-2),2*q01*pow(ball_a1,-2)-2*ball_offset1*pow(ball_a1,-2),
        4*p01*pow(outside_a1,-2)*p02*pow(ball_a0,-2)+8*p04*q00*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*p04*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*p04*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*p04*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)+4*p05*q01*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*p00*pow(outside_a0,-2)*p04*pow(ball_a0,-2)-8*p02*q00*pow(outside_a0,-2)*p04*pow(ball_a0,-2)+4*p02*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*p02*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*p02*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)-4*p03*q01*pow(outside_a0,-2)*p04*pow(ball_a0,-2),4*p01*pow(outside_a1,-2)*p03*pow(ball_a1,-2)+4*p04*q00*pow(outside_a1,-2)*p03*pow(ball_a1,-2)+4*p05*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*p05*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+8*p05*q01*pow(outside_a1,-2)*p03*pow(ball_a1,-2)-4*p05*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*p05*pow(ball_a1,-2)-4*p02*q00*pow(outside_a0,-2)*p05*pow(ball_a1,-2)-4*p03*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*p03*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-8*p03*q01*pow(outside_a0,-2)*p05*pow(ball_a1,-2)+4*p03*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2),
        0,0
    ;
    return out;
}

BallOnCircleSolver::ddtd1ck_t
BallOnCircleSolver::get_ddtd1ck(
    p_t const & p
    ,
    ddtp_t const & ddtp
    ,
    q_t const & q
    ,
    ddtq_t const & ddtq
    ,
    v_t const & v
) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double q00 = q(0);
    double q01 = q(1);
    ddtd1ck_t out;
    out <<
+v(0)*((2*pow(outside_a0,-2))*ddtp(0) + (2*q00*pow(outside_a0,-2))*ddtp(2) + (2*q01*pow(outside_a0,-2))*ddtp(3) + (2*p02*pow(outside_a0,-2))*ddtq(0) + (2*p03*pow(outside_a0,-2))*ddtq(1))+v(1)*((2*pow(outside_a1,-2))*ddtp(1) + (2*q00*pow(outside_a1,-2))*ddtp(4) + (2*q01*pow(outside_a1,-2))*ddtp(5) + (2*p04*pow(outside_a1,-2))*ddtq(0) + (2*p05*pow(outside_a1,-2))*ddtq(1))+v(2)*((2*p03*q00*pow(outside_a0,-2)-2*p02*q01*pow(outside_a0,-2))*ddtp(0) + (2*p05*q00*pow(outside_a1,-2)-2*p04*q01*pow(outside_a1,-2))*ddtp(1) + (2*p03*pow(q00,2)*pow(outside_a0,-2)-2*p00*q01*pow(outside_a0,-2)-4*p02*q00*q01*pow(outside_a0,-2)-2*p03*pow(q01,2)*pow(outside_a0,-2))*ddtp(2) + (2*p00*q00*pow(outside_a0,-2)+2*p02*pow(q00,2)*pow(outside_a0,-2)+4*p03*q00*q01*pow(outside_a0,-2)-2*p02*pow(q01,2)*pow(outside_a0,-2))*ddtp(3) + (2*p05*pow(q00,2)*pow(outside_a1,-2)-2*p01*q01*pow(outside_a1,-2)-4*p04*q00*q01*pow(outside_a1,-2)-2*p05*pow(q01,2)*pow(outside_a1,-2))*ddtp(4) + (2*p01*q00*pow(outside_a1,-2)+2*p04*pow(q00,2)*pow(outside_a1,-2)+4*p05*q00*q01*pow(outside_a1,-2)-2*p04*pow(q01,2)*pow(outside_a1,-2))*ddtp(5) + (2*p03*p00*pow(outside_a0,-2)+4*p03*p02*q00*pow(outside_a0,-2)+2*pow(p03,2)*q01*pow(outside_a0,-2)-2*pow(p02,2)*q01*pow(outside_a0,-2)+2*p05*p01*pow(outside_a1,-2)+4*p05*p04*q00*pow(outside_a1,-2)+2*pow(p05,2)*q01*pow(outside_a1,-2)-2*pow(p04,2)*q01*pow(outside_a1,-2))*ddtq(0) + (2*pow(p03,2)*q00*pow(outside_a0,-2)-2*p02*p00*pow(outside_a0,-2)-2*pow(p02,2)*q00*pow(outside_a0,-2)-4*p02*p03*q01*pow(outside_a0,-2)+2*pow(p05,2)*q00*pow(outside_a1,-2)-2*p04*p01*pow(outside_a1,-2)-2*pow(p04,2)*q00*pow(outside_a1,-2)-4*p04*p05*q01*pow(outside_a1,-2))*ddtq(1)),
+v(0)*0+v(1)*0+v(2)*0,
+v(0)*((-4*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2))*ddtp(4) + (-4*pow(outside_a0,-2)*q01*pow(ball_a1,-2)+4*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2))*ddtp(5) + (-4*pow(outside_a0,-2)*p04*pow(ball_a0,-2))*ddtq(0) + (-4*pow(outside_a0,-2)*p05*pow(ball_a1,-2))*ddtq(1))+v(1)*((4*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2))*ddtp(2) + (4*pow(outside_a1,-2)*q01*pow(ball_a1,-2)-4*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2))*ddtp(3) + (4*pow(outside_a1,-2)*p02*pow(ball_a0,-2))*ddtq(0) + (4*pow(outside_a1,-2)*p03*pow(ball_a1,-2))*ddtq(1))+v(2)*((-4*p05*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p05*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)+4*p04*pow(outside_a0,-2)*q01*pow(ball_a1,-2)-4*p04*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2))*ddtp(0) + (4*p03*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p03*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)-4*p02*pow(outside_a1,-2)*q01*pow(ball_a1,-2)+4*p02*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2))*ddtp(1) + (-4*p01*pow(outside_a1,-2)*q01*pow(ball_a1,-2)+4*p01*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)-4*p04*q00*pow(outside_a1,-2)*q01*pow(ball_a1,-2)+4*p04*q00*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)-4*p05*pow(q01,2)*pow(outside_a1,-2)*pow(ball_a1,-2)+4*p05*q01*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)+4*q01*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)-4*q01*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)+4*pow(q01,2)*pow(outside_a0,-2)*p05*pow(ball_a1,-2)-4*q01*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)+4*p05*pow(q00,2)*pow(outside_a1,-2)*pow(ball_a0,-2)-4*p05*q00*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)-4*p05*pow(q00,2)*pow(outside_a0,-2)*pow(ball_a0,-2)+4*p05*q00*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p04*q01*pow(outside_a1,-2)*q00*pow(ball_a0,-2)+4*p04*q01*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)+4*p04*q00*pow(outside_a0,-2)*q01*pow(ball_a1,-2)-4*p04*q00*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2))*ddtp(2) + (4*p01*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p01*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)+4*p04*pow(q00,2)*pow(outside_a1,-2)*pow(ball_a0,-2)-4*p04*q00*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)+4*p05*q01*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p05*q01*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)-4*pow(q00,2)*pow(outside_a0,-2)*p04*pow(ball_a0,-2)+4*q00*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*q00*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*q00*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)+4*p05*q00*pow(outside_a1,-2)*q01*pow(ball_a1,-2)-4*p05*q00*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)-4*p05*q01*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p05*q01*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p04*pow(q01,2)*pow(outside_a1,-2)*pow(ball_a1,-2)+4*p04*q01*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)+4*p04*pow(q01,2)*pow(outside_a0,-2)*pow(ball_a1,-2)-4*p04*q01*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2))*ddtp(3) + (4*p03*pow(q00,2)*pow(outside_a1,-2)*pow(ball_a0,-2)-4*p03*q00*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)-4*p03*pow(q00,2)*pow(outside_a0,-2)*pow(ball_a0,-2)+4*p03*q00*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p02*q00*pow(outside_a1,-2)*q01*pow(ball_a1,-2)+4*p02*q00*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)+4*p02*q01*pow(outside_a0,-2)*q00*pow(ball_a0,-2)-4*p02*q01*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*q01*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)+4*q01*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)-4*pow(q01,2)*pow(outside_a1,-2)*p03*pow(ball_a1,-2)+4*q01*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)+4*p00*pow(outside_a0,-2)*q01*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2)+4*p02*q00*pow(outside_a0,-2)*q01*pow(ball_a1,-2)-4*p02*q00*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2)+4*p03*pow(q01,2)*pow(outside_a0,-2)*pow(ball_a1,-2)-4*p03*q01*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2))*ddtp(4) + (4*p03*q01*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p03*q01*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)-4*p03*q00*pow(outside_a0,-2)*q01*pow(ball_a1,-2)+4*p03*q00*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2)-4*p02*pow(q01,2)*pow(outside_a1,-2)*pow(ball_a1,-2)+4*p02*q01*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)+4*p02*pow(q01,2)*pow(outside_a0,-2)*pow(ball_a1,-2)-4*p02*q01*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2)+4*pow(q00,2)*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*q00*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*q00*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*q00*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p00*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p02*pow(q00,2)*pow(outside_a0,-2)*pow(ball_a0,-2)+4*p02*q00*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p03*q01*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p03*q01*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2))*ddtp(5) + (4*p03*p01*pow(outside_a1,-2)*pow(ball_a0,-2)+8*p03*p04*q00*pow(outside_a1,-2)*pow(ball_a0,-2)-4*p03*p04*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)+4*p03*p05*q01*pow(outside_a1,-2)*pow(ball_a0,-2)-8*p03*q00*pow(outside_a0,-2)*p04*pow(ball_a0,-2)+4*p03*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*p03*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*p03*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)-4*p02*p04*pow(outside_a1,-2)*q01*pow(ball_a1,-2)+4*p02*p04*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)+4*p02*q01*pow(outside_a0,-2)*p04*pow(ball_a0,-2)+8*p05*q00*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*p05*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*p05*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*p05*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p05*p00*pow(outside_a0,-2)*pow(ball_a0,-2)-8*p05*p02*q00*pow(outside_a0,-2)*pow(ball_a0,-2)+4*p05*p02*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p05*p03*q01*pow(outside_a0,-2)*pow(ball_a0,-2)-4*p04*q01*pow(outside_a1,-2)*p02*pow(ball_a0,-2)+4*p04*p02*pow(outside_a0,-2)*q01*pow(ball_a1,-2)-4*p04*p02*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2))*ddtq(0) + (4*p03*p05*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p03*p05*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)-4*p03*q00*pow(outside_a0,-2)*p05*pow(ball_a1,-2)-4*p02*p01*pow(outside_a1,-2)*pow(ball_a1,-2)-4*p02*p04*q00*pow(outside_a1,-2)*pow(ball_a1,-2)-8*p02*p05*q01*pow(outside_a1,-2)*pow(ball_a1,-2)+4*p02*p05*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)+4*p02*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)-4*p02*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)+8*p02*q01*pow(outside_a0,-2)*p05*pow(ball_a1,-2)-4*p02*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2)+4*p05*q00*pow(outside_a1,-2)*p03*pow(ball_a1,-2)-4*p05*p03*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p05*p03*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p04*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)+4*p04*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)-8*p04*q01*pow(outside_a1,-2)*p03*pow(ball_a1,-2)+4*p04*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)+4*p04*p00*pow(outside_a0,-2)*pow(ball_a1,-2)+4*p04*p02*q00*pow(outside_a0,-2)*pow(ball_a1,-2)+8*p04*p03*q01*pow(outside_a0,-2)*pow(ball_a1,-2)-4*p04*p03*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2))*ddtq(1)),
+v(0)*((2*pow(outside_a1,-2))*ddtp(1) + (2*q00*pow(outside_a1,-2))*ddtp(4) + (2*q01*pow(outside_a1,-2))*ddtp(5) + (2*p04*pow(outside_a1,-2))*ddtq(0) + (2*p05*pow(outside_a1,-2))*ddtq(1))+v(1)*((-2*pow(outside_a0,-2))*ddtp(0) + (-2*q00*pow(outside_a0,-2))*ddtp(2) + (-2*q01*pow(outside_a0,-2))*ddtp(3) + (-2*p02*pow(outside_a0,-2))*ddtq(0) + (-2*p03*pow(outside_a0,-2))*ddtq(1))+v(2)*((2*pow(outside_a0,-2)*p04*q01-2*pow(outside_a0,-2)*p05*q00)*ddtp(0) + (-2*pow(outside_a1,-2)*p02*q01+2*pow(outside_a1,-2)*p03*q00)*ddtp(1) + (-2*p01*pow(outside_a1,-2)*q01-2*p04*q00*pow(outside_a1,-2)*q01-2*p05*pow(q01,2)*pow(outside_a1,-2)+2*q00*pow(outside_a0,-2)*p04*q01-2*pow(q00,2)*pow(outside_a0,-2)*p05)*ddtp(2) + (2*p01*pow(outside_a1,-2)*q00+2*p04*pow(q00,2)*pow(outside_a1,-2)+2*p05*q01*pow(outside_a1,-2)*q00+2*pow(q01,2)*pow(outside_a0,-2)*p04-2*q01*pow(outside_a0,-2)*p05*q00)*ddtp(3) + (-2*q00*pow(outside_a1,-2)*p02*q01+2*pow(q00,2)*pow(outside_a1,-2)*p03+2*p00*pow(outside_a0,-2)*q01+2*p02*q00*pow(outside_a0,-2)*q01+2*p03*pow(q01,2)*pow(outside_a0,-2))*ddtp(4) + (-2*pow(q01,2)*pow(outside_a1,-2)*p02+2*q01*pow(outside_a1,-2)*p03*q00-2*p00*pow(outside_a0,-2)*q00-2*p02*pow(q00,2)*pow(outside_a0,-2)-2*p03*q01*pow(outside_a0,-2)*q00)*ddtp(5) + (2*p01*pow(outside_a1,-2)*p03-2*p04*pow(outside_a1,-2)*p02*q01+4*p04*q00*pow(outside_a1,-2)*p03+2*p05*q01*pow(outside_a1,-2)*p03-2*p00*pow(outside_a0,-2)*p05+2*p02*pow(outside_a0,-2)*p04*q01-4*p02*q00*pow(outside_a0,-2)*p05-2*p03*q01*pow(outside_a0,-2)*p05)*ddtq(0) + (-2*p01*pow(outside_a1,-2)*p02-2*p04*q00*pow(outside_a1,-2)*p02-4*p05*q01*pow(outside_a1,-2)*p02+2*p05*pow(outside_a1,-2)*p03*q00+2*p00*pow(outside_a0,-2)*p04+2*p02*q00*pow(outside_a0,-2)*p04+4*p03*q01*pow(outside_a0,-2)*p04-2*p03*pow(outside_a0,-2)*p05*q00)*ddtq(1));
    return out;
}

BallOnCircleSolver::ddtd2c_t
BallOnCircleSolver::get_ddtd2c(
    p_t const & p
    ,
    ddtp_t const & ddtp
    ,
    q_t const & q
    ,
    ddtq_t const & ddtq
) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double q00 = q(0);
    double q01 = q(1);
    ddtd2c_t out;
    out <<
        (2*p02*pow(outside_a0,-2))*ddtp(0) + (2*p04*pow(outside_a1,-2))*ddtp(1) + (2*p00*pow(outside_a0,-2)+4*p02*q00*pow(outside_a0,-2)+2*p03*q01*pow(outside_a0,-2))*ddtp(2) + (2*p02*q01*pow(outside_a0,-2))*ddtp(3) + (2*p01*pow(outside_a1,-2)+4*p04*q00*pow(outside_a1,-2)+2*p05*q01*pow(outside_a1,-2))*ddtp(4) + (2*p04*q01*pow(outside_a1,-2))*ddtp(5) + (2*pow(p02,2)*pow(outside_a0,-2)+2*pow(p04,2)*pow(outside_a1,-2))*ddtq(0) + (2*p02*p03*pow(outside_a0,-2)+2*p04*p05*pow(outside_a1,-2))*ddtq(1), (2*p03*pow(outside_a0,-2))*ddtp(0) + (2*p05*pow(outside_a1,-2))*ddtp(1) + (2*q00*p03*pow(outside_a0,-2))*ddtp(2) + (2*p00*pow(outside_a0,-2)+2*p02*q00*pow(outside_a0,-2)+4*p03*q01*pow(outside_a0,-2))*ddtp(3) + (2*q00*p05*pow(outside_a1,-2))*ddtp(4) + (2*p01*pow(outside_a1,-2)+2*p04*q00*pow(outside_a1,-2)+4*p05*q01*pow(outside_a1,-2))*ddtp(5) + (2*p02*p03*pow(outside_a0,-2)+2*p04*p05*pow(outside_a1,-2))*ddtq(0) + (2*pow(p03,2)*pow(outside_a0,-2)+2*pow(p05,2)*pow(outside_a1,-2))*ddtq(1),
        (2*pow(ball_a0,-2))*ddtq(0), (2*pow(ball_a1,-2))*ddtq(1),
        (-4*pow(outside_a0,-2)*p04*pow(ball_a0,-2))*ddtp(0) + (4*pow(outside_a1,-2)*p02*pow(ball_a0,-2))*ddtp(1) + (4*p01*pow(outside_a1,-2)*pow(ball_a0,-2)+8*p04*q00*pow(outside_a1,-2)*pow(ball_a0,-2)-4*p04*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)+4*p05*q01*pow(outside_a1,-2)*pow(ball_a0,-2)-8*q00*pow(outside_a0,-2)*p04*pow(ball_a0,-2)+4*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-4*pow(outside_a0,-2)*p05*q01*pow(ball_a1,-2)+4*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2))*ddtp(2) + (4*p04*pow(outside_a1,-2)*q01*pow(ball_a1,-2)-4*p04*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)-4*q01*pow(outside_a0,-2)*p04*pow(ball_a0,-2))*ddtp(3) + (8*q00*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+4*pow(outside_a1,-2)*p03*q01*pow(ball_a1,-2)-4*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*pow(ball_a0,-2)-8*p02*q00*pow(outside_a0,-2)*pow(ball_a0,-2)+4*p02*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2)-4*p03*q01*pow(outside_a0,-2)*pow(ball_a0,-2))*ddtp(4) + (4*q01*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*p02*pow(outside_a0,-2)*q01*pow(ball_a1,-2)+4*p02*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2))*ddtp(5) + (8*p04*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-8*p02*pow(outside_a0,-2)*p04*pow(ball_a0,-2))*ddtq(0) + (4*p04*pow(outside_a1,-2)*p03*pow(ball_a1,-2)+4*p05*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*p02*pow(outside_a0,-2)*p05*pow(ball_a1,-2)-4*p03*pow(outside_a0,-2)*p04*pow(ball_a0,-2))*ddtq(1), (-4*pow(outside_a0,-2)*p05*pow(ball_a1,-2))*ddtp(0) + (4*pow(outside_a1,-2)*p03*pow(ball_a1,-2))*ddtp(1) + (4*p05*pow(outside_a1,-2)*q00*pow(ball_a0,-2)-4*p05*pow(outside_a1,-2)*ball_offset0*pow(ball_a0,-2)-4*q00*pow(outside_a0,-2)*p05*pow(ball_a1,-2))*ddtp(2) + (4*p01*pow(outside_a1,-2)*pow(ball_a1,-2)+4*p04*q00*pow(outside_a1,-2)*pow(ball_a1,-2)+8*p05*q01*pow(outside_a1,-2)*pow(ball_a1,-2)-4*p05*pow(outside_a1,-2)*ball_offset1*pow(ball_a1,-2)-4*pow(outside_a0,-2)*p04*q00*pow(ball_a0,-2)+4*pow(outside_a0,-2)*p04*ball_offset0*pow(ball_a0,-2)-8*q01*pow(outside_a0,-2)*p05*pow(ball_a1,-2)+4*pow(outside_a0,-2)*p05*ball_offset1*pow(ball_a1,-2))*ddtp(3) + (4*q00*pow(outside_a1,-2)*p03*pow(ball_a1,-2)-4*p03*pow(outside_a0,-2)*q00*pow(ball_a0,-2)+4*p03*pow(outside_a0,-2)*ball_offset0*pow(ball_a0,-2))*ddtp(4) + (4*pow(outside_a1,-2)*p02*q00*pow(ball_a0,-2)-4*pow(outside_a1,-2)*p02*ball_offset0*pow(ball_a0,-2)+8*q01*pow(outside_a1,-2)*p03*pow(ball_a1,-2)-4*pow(outside_a1,-2)*p03*ball_offset1*pow(ball_a1,-2)-4*p00*pow(outside_a0,-2)*pow(ball_a1,-2)-4*p02*q00*pow(outside_a0,-2)*pow(ball_a1,-2)-8*p03*q01*pow(outside_a0,-2)*pow(ball_a1,-2)+4*p03*pow(outside_a0,-2)*ball_offset1*pow(ball_a1,-2))*ddtp(5) + (4*p04*pow(outside_a1,-2)*p03*pow(ball_a1,-2)+4*p05*pow(outside_a1,-2)*p02*pow(ball_a0,-2)-4*p02*pow(outside_a0,-2)*p05*pow(ball_a1,-2)-4*p03*pow(outside_a0,-2)*p04*pow(ball_a0,-2))*ddtq(0) + (8*p05*pow(outside_a1,-2)*p03*pow(ball_a1,-2)-8*p03*pow(outside_a0,-2)*p05*pow(ball_a1,-2))*ddtq(1),
        0, 0;
    return out;
}

