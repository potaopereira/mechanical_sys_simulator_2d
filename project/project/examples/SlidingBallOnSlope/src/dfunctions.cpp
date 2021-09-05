// SlidingBallOnSlopeSolver
#include "SlidingBallOnSlope/SlidingBallOnSlopeSolver.hpp"

SlidingBallOnSlopeSolver::c_t
SlidingBallOnSlopeSolver::get_c(p_t const & p, q_t const & q) const {
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
        p00*slope+p02*q00*slope+p03*q01*slope+p01+p04*q00+p05*q01,
        pow(q00,2)*pow(radius1,-2)+pow(q01,2)*pow(radius2,-2)-1,
        2*p02*q00*pow(radius1,-2)+2*p03*q01*pow(radius2,-2)-2*slope*p04*q00*pow(radius1,-2)-2*slope*p05*q01*pow(radius2,-2);
    return out;
}

SlidingBallOnSlopeSolver::cextra_t
SlidingBallOnSlopeSolver::get_cextra(p_t const & p, q_t const & q) const {
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
        p00*slope+p02*q00*slope+p03*q01*slope+p01+p04*q00+p05*q01,
        pow(q00,2)*pow(radius1,-2)+pow(q01,2)*pow(radius2,-2)-1,
        2*p02*q00*pow(radius1,-2)+2*p03*q01*pow(radius2,-2)-2*slope*p04*q00*pow(radius1,-2)-2*slope*p05*q01*pow(radius2,-2),
        pow(p02,2)+pow(p03,2)-1,
        pow(p04,2)+pow(p05,2)-1,
        p02*p04+p03*p05;
    return out;
}

SlidingBallOnSlopeSolver::d1cextra_t
SlidingBallOnSlopeSolver::get_d1cextra(p_t const & p, q_t const & q) const {
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
        slope, 1, q00*slope, q01*slope, q00, q01, 
        0, 0, 0, 0, 0, 0, 
        0, 0, 2*q00*pow(radius1,-2), 2*q01*pow(radius2,-2), -2*slope*q00*pow(radius1,-2), -2*slope*q01*pow(radius2,-2), 
        0, 0, 2*p02, 2*p03, 0, 0, 
        0, 0, 0, 0, 2*p04, 2*p05, 
        0, 0, p04, p05, p02, p03;
    return out;
}

SlidingBallOnSlopeSolver::d2cextra_t
SlidingBallOnSlopeSolver::get_d2cextra(p_t const & p, q_t const & q) const {
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
        p02*slope+p04, p03*slope+p05, 
        2*q00*pow(radius1,-2), 2*q01*pow(radius2,-2), 
        2*p02*pow(radius1,-2)-2*slope*p04*pow(radius1,-2), 2*p03*pow(radius2,-2)-2*slope*p05*pow(radius2,-2), 
        0, 0, 
        0, 0, 
        0, 0;
    return out;
}

SlidingBallOnSlopeSolver::d1ck_t
SlidingBallOnSlopeSolver::get_d1ck(p_t const & p, q_t const & q) const {
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
        slope, 1, p03*q00*slope-p02*q01*slope+p05*q00-p04*q01, 
        0, 0, 0, 
        0, 0, 2*p03*q00*pow(radius1,-2)-2*p02*q01*pow(radius2,-2)-2*p05*slope*q00*pow(radius1,-2)+2*p04*slope*q01*pow(radius2,-2);
    return out;
}

SlidingBallOnSlopeSolver::d1c_t
SlidingBallOnSlopeSolver::get_d1c(p_t const & p, q_t const & q) const {
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
        slope, 1, p03*q00*slope-p02*q01*slope+p05*q00-p04*q01, 0, 0, 0, 
        0, 0, 0, 0, 0, 2*p03*q00*pow(radius1,-2)-2*p02*q01*pow(radius2,-2)-2*p05*slope*q00*pow(radius1,-2)+2*p04*slope*q01*pow(radius2,-2), 
        0, 0, 2*p03*q00*pow(radius1,-2)-2*p02*q01*pow(radius2,-2)-2*p05*slope*q00*pow(radius1,-2)+2*p04*slope*q01*pow(radius2,-2), 0, 0, 0;
    return out;
}

SlidingBallOnSlopeSolver::d2c_t
SlidingBallOnSlopeSolver::get_d2c(p_t const & p, q_t const & q) const {
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
        p02*slope+p04,p03*slope+p05,
        2*q00*pow(radius1,-2),2*q01*pow(radius2,-2),
        2*p02*pow(radius1,-2)-2*slope*p04*pow(radius1,-2),2*p03*pow(radius2,-2)-2*slope*p05*pow(radius2,-2)
    ;
    return out;
}

SlidingBallOnSlopeSolver::ddtd1ck_t
SlidingBallOnSlopeSolver::get_ddtd1ck(
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
+v(0)*0+v(1)*0+v(2)*((-q01*slope)*ddtp(2) + (q00*slope)*ddtp(3) + (-q01)*ddtp(4) + (q00)*ddtp(5) + (p03*slope+p05)*ddtq(0) + (-p02*slope-p04)*ddtq(1)),
+v(0)*0+v(1)*0+v(2)*0,
+v(0)*0+v(1)*0+v(2)*((-2*q01*pow(radius2,-2))*ddtp(2) + (2*q00*pow(radius1,-2))*ddtp(3) + (2*slope*q01*pow(radius2,-2))*ddtp(4) + (-2*slope*q00*pow(radius1,-2))*ddtp(5) + (2*p03*pow(radius1,-2)-2*p05*slope*pow(radius1,-2))*ddtq(0) + (-2*p02*pow(radius2,-2)+2*p04*slope*pow(radius2,-2))*ddtq(1));
    return out;
}

SlidingBallOnSlopeSolver::ddtd2c_t
SlidingBallOnSlopeSolver::get_ddtd2c(
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
        (slope)*ddtp(2) + (1)*ddtp(4), (slope)*ddtp(3) + (1)*ddtp(5),
        (2*pow(radius1,-2))*ddtq(0), (2*pow(radius2,-2))*ddtq(1),
        (2*pow(radius1,-2))*ddtp(2) + (-2*slope*pow(radius1,-2))*ddtp(4), (2*pow(radius2,-2))*ddtp(3) + (-2*slope*pow(radius2,-2))*ddtp(5);
    return out;
}
