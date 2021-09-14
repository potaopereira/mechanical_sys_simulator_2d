// BallsOnSlopesSolver
#include "BallsOnSlopes/BallsOnSlopesSolver.hpp"

BallsOnSlopesSolver::c_t
BallsOnSlopesSolver::get_c(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double p10 = p(6);
    double p11 = p(7);
    double p12 = p(8);
    double p13 = p(9);
    double p14 = p(10);
    double p15 = p(11);
    double q00 = q(0);
    double q01 = q(1);
    double q10 = q(2);
    double q11 = q(3);
    c_t out;
    out << 
        pow(p00,2)-2*p00*p10+pow(p10,2)+pow(p01,2)-2*p01*p11+pow(p11,2)-l_squared,
        p00*slope_1+p02*q00*slope_1+p03*q01*slope_1+p01+p04*q00+p05*q01,
        pow(q00,2)*pow(radius1_1,-2)+pow(q01,2)*pow(radius1_2,-2)-1,
        2*p02*q00*pow(radius1_1,-2)+2*p03*q01*pow(radius1_2,-2)-2*slope_1*p04*q00*pow(radius1_1,-2)-2*slope_1*p05*q01*pow(radius1_2,-2),
        p10+p12*q10+p13*q11+p11*slope_2+p14*q10*slope_2+p15*q11*slope_2,
        pow(q10,2)*pow(radius2_1,-2)+pow(q11,2)*pow(radius2_2,-2)-1,
        2*slope_2*p12*q10*pow(radius2_1,-2)+2*slope_2*p13*q11*pow(radius2_2,-2)-2*p14*q10*pow(radius2_1,-2)-2*p15*q11*pow(radius2_2,-2),
        0,
        0;
    return out;
}

BallsOnSlopesSolver::cextra_t
BallsOnSlopesSolver::get_cextra(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double p10 = p(6);
    double p11 = p(7);
    double p12 = p(8);
    double p13 = p(9);
    double p14 = p(10);
    double p15 = p(11);
    double q00 = q(0);
    double q01 = q(1);
    double q10 = q(2);
    double q11 = q(3);
    cextra_t out;
    out << 
        pow(p00,2)-2*p00*p10+pow(p10,2)+pow(p01,2)-2*p01*p11+pow(p11,2)-l_squared,
        p00*slope_1+p02*q00*slope_1+p03*q01*slope_1+p01+p04*q00+p05*q01,
        pow(q00,2)*pow(radius1_1,-2)+pow(q01,2)*pow(radius1_2,-2)-1,
        2*p02*q00*pow(radius1_1,-2)+2*p03*q01*pow(radius1_2,-2)-2*slope_1*p04*q00*pow(radius1_1,-2)-2*slope_1*p05*q01*pow(radius1_2,-2),
        p10+p12*q10+p13*q11+p11*slope_2+p14*q10*slope_2+p15*q11*slope_2,
        pow(q10,2)*pow(radius2_1,-2)+pow(q11,2)*pow(radius2_2,-2)-1,
        2*slope_2*p12*q10*pow(radius2_1,-2)+2*slope_2*p13*q11*pow(radius2_2,-2)-2*p14*q10*pow(radius2_1,-2)-2*p15*q11*pow(radius2_2,-2),
        pow(p02,2)+pow(p03,2)-1,
        pow(p04,2)+pow(p05,2)-1,
        p02*p04+p03*p05,
        pow(p12,2)+pow(p13,2)-1,
        pow(p14,2)+pow(p15,2)-1,
        p12*p14+p13*p15;
    return out;
}

BallsOnSlopesSolver::d1cextra_t
BallsOnSlopesSolver::get_d1cextra(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double p10 = p(6);
    double p11 = p(7);
    double p12 = p(8);
    double p13 = p(9);
    double p14 = p(10);
    double p15 = p(11);
    double q00 = q(0);
    double q01 = q(1);
    double q10 = q(2);
    double q11 = q(3);
    d1cextra_t out;
    out << 
        2*p00-2*p10, 2*p01-2*p11, 0, 0, 0, 0, -2*p00+2*p10, -2*p01+2*p11, 0, 0, 0, 0, 
        slope_1, 1, q00*slope_1, q01*slope_1, q00, q01, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 2*q00*pow(radius1_1,-2), 2*q01*pow(radius1_2,-2), -2*slope_1*q00*pow(radius1_1,-2), -2*slope_1*q01*pow(radius1_2,-2), 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 1, slope_2, q10, q11, q10*slope_2, q11*slope_2, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 2*slope_2*q10*pow(radius2_1,-2), 2*slope_2*q11*pow(radius2_2,-2), -2*q10*pow(radius2_1,-2), -2*q11*pow(radius2_2,-2), 
        0, 0, 2*p02, 2*p03, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 2*p04, 2*p05, 0, 0, 0, 0, 0, 0, 
        0, 0, p04, p05, p02, p03, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 2*p12, 2*p13, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2*p14, 2*p15, 
        0, 0, 0, 0, 0, 0, 0, 0, p14, p15, p12, p13;
    return out;
}

BallsOnSlopesSolver::d2cextra_t
BallsOnSlopesSolver::get_d2cextra(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double p10 = p(6);
    double p11 = p(7);
    double p12 = p(8);
    double p13 = p(9);
    double p14 = p(10);
    double p15 = p(11);
    double q00 = q(0);
    double q01 = q(1);
    double q10 = q(2);
    double q11 = q(3);
    d2cextra_t out;
    out << 
        0, 0, 0, 0, 
        p02*slope_1+p04, p03*slope_1+p05, 0, 0, 
        2*q00*pow(radius1_1,-2), 2*q01*pow(radius1_2,-2), 0, 0, 
        2*p02*pow(radius1_1,-2)-2*slope_1*p04*pow(radius1_1,-2), 2*p03*pow(radius1_2,-2)-2*slope_1*p05*pow(radius1_2,-2), 0, 0, 
        0, 0, p12+p14*slope_2, p13+p15*slope_2, 
        0, 0, 2*q10*pow(radius2_1,-2), 2*q11*pow(radius2_2,-2), 
        0, 0, 2*slope_2*p12*pow(radius2_1,-2)-2*p14*pow(radius2_1,-2), 2*slope_2*p13*pow(radius2_2,-2)-2*p15*pow(radius2_2,-2), 
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        0, 0, 0, 0;
    return out;
}

BallsOnSlopesSolver::d1ck_t
BallsOnSlopesSolver::get_d1ck(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double p10 = p(6);
    double p11 = p(7);
    double p12 = p(8);
    double p13 = p(9);
    double p14 = p(10);
    double p15 = p(11);
    double q00 = q(0);
    double q01 = q(1);
    double q10 = q(2);
    double q11 = q(3);
    d1ck_t out;
    out <<
        2*p00-2*p10, 2*p01-2*p11, 0, -2*p00+2*p10, -2*p01+2*p11, 0, 
        slope_1, 1, p03*q00*slope_1-p02*q01*slope_1+p05*q00-p04*q01, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 
        0, 0, 2*p03*q00*pow(radius1_1,-2)-2*p02*q01*pow(radius1_2,-2)-2*p05*slope_1*q00*pow(radius1_1,-2)+2*p04*slope_1*q01*pow(radius1_2,-2), 0, 0, 0, 
        0, 0, 0, 1, slope_2, p13*q10-p12*q11+p15*q10*slope_2-p14*q11*slope_2, 
        0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 2*p13*slope_2*q10*pow(radius2_1,-2)-2*p12*slope_2*q11*pow(radius2_2,-2)-2*p15*q10*pow(radius2_1,-2)+2*p14*q11*pow(radius2_2,-2), 
        1, -slope_1, -p02*q01+p03*q00+slope_1*p04*q01-slope_1*p05*q00, 0, 0, 0, 
        0, 0, 0, slope_2, -1, -slope_2*p12*q11+slope_2*p13*q10+p14*q11-p15*q10;
    return out;
}

BallsOnSlopesSolver::d1c_t
BallsOnSlopesSolver::get_d1c(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double p10 = p(6);
    double p11 = p(7);
    double p12 = p(8);
    double p13 = p(9);
    double p14 = p(10);
    double p15 = p(11);
    double q00 = q(0);
    double q01 = q(1);
    double q10 = q(2);
    double q11 = q(3);
    d1c_t out;
    out <<
        2*p00-2*p10, 2*p01-2*p11, 0, 0, 0, 0, -2*p00+2*p10, -2*p01+2*p11, 0, 0, 0, 0, 
        slope_1, 1, q00*slope_1, q01*slope_1, q00, q01, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 2*q00*pow(radius1_1,-2), 2*q01*pow(radius1_2,-2), -2*slope_1*q00*pow(radius1_1,-2), -2*slope_1*q01*pow(radius1_2,-2), 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 1, slope_2, q10, q11, q10*slope_2, q11*slope_2, 
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 0, 0, 2*slope_2*q10*pow(radius2_1,-2), 2*slope_2*q11*pow(radius2_2,-2), -2*q10*pow(radius2_1,-2), -2*q11*pow(radius2_2,-2), 
        1, -slope_1, q00, q01, -slope_1*q00, -slope_1*q01, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, slope_2, -1, slope_2*q10, slope_2*q11, -q10, -q11;
    return out;
}

BallsOnSlopesSolver::d2c_t
BallsOnSlopesSolver::get_d2c(p_t const & p, q_t const & q) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    double p10 = p(6);
    double p11 = p(7);
    double p12 = p(8);
    double p13 = p(9);
    double p14 = p(10);
    double p15 = p(11);
    double q00 = q(0);
    double q01 = q(1);
    double q10 = q(2);
    double q11 = q(3);
    d2c_t out;
    out <<
        0,0,0,0,
        p02*slope_1+p04,p03*slope_1+p05,0,0,
        2*q00*pow(radius1_1,-2),2*q01*pow(radius1_2,-2),0,0,
        2*p02*pow(radius1_1,-2)-2*slope_1*p04*pow(radius1_1,-2),2*p03*pow(radius1_2,-2)-2*slope_1*p05*pow(radius1_2,-2),0,0,
        0,0,p12+p14*slope_2,p13+p15*slope_2,
        0,0,2*q10*pow(radius2_1,-2),2*q11*pow(radius2_2,-2),
        0,0,2*slope_2*p12*pow(radius2_1,-2)-2*p14*pow(radius2_1,-2),2*slope_2*p13*pow(radius2_2,-2)-2*p15*pow(radius2_2,-2),
        0,0,0,0,
        0,0,0,0
    ;
    return out;
}

BallsOnSlopesSolver::ddtd1ck_t
BallsOnSlopesSolver::get_ddtd1ck(
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
    double p10 = p(6);
    double p11 = p(7);
    double p12 = p(8);
    double p13 = p(9);
    double p14 = p(10);
    double p15 = p(11);
    double q00 = q(0);
    double q01 = q(1);
    double q10 = q(2);
    double q11 = q(3);
    ddtd1ck_t out;
    out <<
+v(0)*((2)*ddtp(0) + (-2)*ddtp(6))+v(1)*((2)*ddtp(1) + (-2)*ddtp(7))+v(2)*0+v(3)*((-2)*ddtp(0) + (2)*ddtp(6))+v(4)*((-2)*ddtp(1) + (2)*ddtp(7))+v(5)*0,
+v(0)*0+v(1)*0+v(2)*((-q01*slope_1)*ddtp(2) + (q00*slope_1)*ddtp(3) + (-q01)*ddtp(4) + (q00)*ddtp(5) + (p03*slope_1+p05)*ddtq(0) + (-p02*slope_1-p04)*ddtq(1))+v(3)*0+v(4)*0+v(5)*0,
+v(0)*0+v(1)*0+v(2)*0+v(3)*0+v(4)*0+v(5)*0,
+v(0)*0+v(1)*0+v(2)*((-2*q01*pow(radius1_2,-2))*ddtp(2) + (2*q00*pow(radius1_1,-2))*ddtp(3) + (2*slope_1*q01*pow(radius1_2,-2))*ddtp(4) + (-2*slope_1*q00*pow(radius1_1,-2))*ddtp(5) + (2*p03*pow(radius1_1,-2)-2*p05*slope_1*pow(radius1_1,-2))*ddtq(0) + (-2*p02*pow(radius1_2,-2)+2*p04*slope_1*pow(radius1_2,-2))*ddtq(1))+v(3)*0+v(4)*0+v(5)*0,
+v(0)*0+v(1)*0+v(2)*0+v(3)*0+v(4)*0+v(5)*((-q11)*ddtp(8) + (q10)*ddtp(9) + (-q11*slope_2)*ddtp(10) + (q10*slope_2)*ddtp(11) + (p13+p15*slope_2)*ddtq(2) + (-p12-p14*slope_2)*ddtq(3)),
+v(0)*0+v(1)*0+v(2)*0+v(3)*0+v(4)*0+v(5)*0,
+v(0)*0+v(1)*0+v(2)*0+v(3)*0+v(4)*0+v(5)*((-2*slope_2*q11*pow(radius2_2,-2))*ddtp(8) + (2*slope_2*q10*pow(radius2_1,-2))*ddtp(9) + (2*q11*pow(radius2_2,-2))*ddtp(10) + (-2*q10*pow(radius2_1,-2))*ddtp(11) + (2*p13*slope_2*pow(radius2_1,-2)-2*p15*pow(radius2_1,-2))*ddtq(2) + (-2*p12*slope_2*pow(radius2_2,-2)+2*p14*pow(radius2_2,-2))*ddtq(3)),
+v(0)*0+v(1)*0+v(2)*((-q01)*ddtp(2) + (q00)*ddtp(3) + (slope_1*q01)*ddtp(4) + (-slope_1*q00)*ddtp(5) + (p03-slope_1*p05)*ddtq(0) + (-p02+slope_1*p04)*ddtq(1))+v(3)*0+v(4)*0+v(5)*0,
+v(0)*0+v(1)*0+v(2)*0+v(3)*0+v(4)*0+v(5)*((-slope_2*q11)*ddtp(8) + (slope_2*q10)*ddtp(9) + (q11)*ddtp(10) + (-q10)*ddtp(11) + (slope_2*p13-p15)*ddtq(2) + (-slope_2*p12+p14)*ddtq(3));
    return out;
}

BallsOnSlopesSolver::ddtd2c_t
BallsOnSlopesSolver::get_ddtd2c(
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
    double p10 = p(6);
    double p11 = p(7);
    double p12 = p(8);
    double p13 = p(9);
    double p14 = p(10);
    double p15 = p(11);
    double q00 = q(0);
    double q01 = q(1);
    double q10 = q(2);
    double q11 = q(3);
    ddtd2c_t out;
    out <<
        0, 0, 0, 0,
        (slope_1)*ddtp(2) + (1)*ddtp(4), (slope_1)*ddtp(3) + (1)*ddtp(5), 0, 0,
        (2*pow(radius1_1,-2))*ddtq(0), (2*pow(radius1_2,-2))*ddtq(1), 0, 0,
        (2*pow(radius1_1,-2))*ddtp(2) + (-2*slope_1*pow(radius1_1,-2))*ddtp(4), (2*pow(radius1_2,-2))*ddtp(3) + (-2*slope_1*pow(radius1_2,-2))*ddtp(5), 0, 0,
        0, 0, (1)*ddtp(8) + (slope_2)*ddtp(10), (1)*ddtp(9) + (slope_2)*ddtp(11),
        0, 0, (2*pow(radius2_1,-2))*ddtq(2), (2*pow(radius2_2,-2))*ddtq(3),
        0, 0, (2*slope_2*pow(radius2_1,-2))*ddtp(8) + (-2*pow(radius2_1,-2))*ddtp(10), (2*slope_2*pow(radius2_2,-2))*ddtp(9) + (-2*pow(radius2_2,-2))*ddtp(11),
        0, 0, 0, 0,
        0, 0, 0, 0;
    return out;
}

