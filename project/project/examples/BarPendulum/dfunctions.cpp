// BarPendulumSolver
#include "BarPendulum/BarPendulumSolver.hpp"

BarPendulumSolver::c_t
BarPendulumSolver::get_c(p_t const & p) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    c_t out;
    out << 
        pow(p00,2)-2*p00*p02*length1_bar-2*p00*p1[0]+pow(p02,2)*pow(length1_bar,2)+2*p02*length1_bar*p1[0]+pow(p1[0],2)+pow(p01,2)-2*p01*p04*length1_bar-2*p01*p1[1]+pow(p04,2)*pow(length1_bar,2)+2*p04*length1_bar*p1[1]+pow(p1[1],2)-arm_1_squared,
        pow(p00,2)+2*p00*p02*length2_bar-2*p00*p2[0]+pow(p02,2)*pow(length2_bar,2)-2*p02*length2_bar*p2[0]+pow(p2[0],2)+pow(p01,2)+2*p01*p04*length2_bar-2*p01*p2[1]+pow(p04,2)*pow(length2_bar,2)-2*p04*length2_bar*p2[1]+pow(p2[1],2)-arm_2_squared;
    return out;
}

BarPendulumSolver::cextra_t
BarPendulumSolver::get_cextra(p_t const & p) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    cextra_t out;
    out << 
        pow(p00,2)-2*p00*p02*length1_bar-2*p00*p1[0]+pow(p02,2)*pow(length1_bar,2)+2*p02*length1_bar*p1[0]+pow(p1[0],2)+pow(p01,2)-2*p01*p04*length1_bar-2*p01*p1[1]+pow(p04,2)*pow(length1_bar,2)+2*p04*length1_bar*p1[1]+pow(p1[1],2)-arm_1_squared,
        pow(p00,2)+2*p00*p02*length2_bar-2*p00*p2[0]+pow(p02,2)*pow(length2_bar,2)-2*p02*length2_bar*p2[0]+pow(p2[0],2)+pow(p01,2)+2*p01*p04*length2_bar-2*p01*p2[1]+pow(p04,2)*pow(length2_bar,2)-2*p04*length2_bar*p2[1]+pow(p2[1],2)-arm_2_squared,
        pow(p02,2)+pow(p03,2)-1,
        pow(p04,2)+pow(p05,2)-1,
        p02*p04+p03*p05;
    return out;
}

BarPendulumSolver::d1cextra_t
BarPendulumSolver::get_d1cextra(p_t const & p) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    d1cextra_t out;
    out << 
        2*p00-2*p02*length1_bar-2*p1[0], 2*p01-2*p04*length1_bar-2*p1[1], -2*p00*length1_bar+2*p02*pow(length1_bar,2)+2*length1_bar*p1[0], 0, -2*p01*length1_bar+2*p04*pow(length1_bar,2)+2*length1_bar*p1[1], 0, 
        2*p00+2*p02*length2_bar-2*p2[0], 2*p01+2*p04*length2_bar-2*p2[1], 2*p00*length2_bar+2*p02*pow(length2_bar,2)-2*length2_bar*p2[0], 0, 2*p01*length2_bar+2*p04*pow(length2_bar,2)-2*length2_bar*p2[1], 0, 
        0, 0, 2*p02, 2*p03, 0, 0, 
        0, 0, 0, 0, 2*p04, 2*p05, 
        0, 0, p04, p05, p02, p03;
    return out;
}

BarPendulumSolver::d1ck_t
BarPendulumSolver::get_d1ck(p_t const & p) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    d1ck_t out;
    out <<
        2*p00-2*p02*length1_bar-2*p1[0], 2*p01-2*p04*length1_bar-2*p1[1], -2*p03*p00*length1_bar+2*p03*p02*pow(length1_bar,2)+2*p03*length1_bar*p1[0]-2*p05*p01*length1_bar+2*p05*p04*pow(length1_bar,2)+2*p05*length1_bar*p1[1], 
        2*p00+2*p02*length2_bar-2*p2[0], 2*p01+2*p04*length2_bar-2*p2[1], 2*p03*p00*length2_bar+2*p03*p02*pow(length2_bar,2)-2*p03*length2_bar*p2[0]+2*p05*p01*length2_bar+2*p05*p04*pow(length2_bar,2)-2*p05*length2_bar*p2[1];
    return out;
}

BarPendulumSolver::d1c_t
BarPendulumSolver::get_d1c(p_t const & p) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    d1c_t out;
    out <<
        2*p00-2*p02*length1_bar-2*p1[0], 2*p01-2*p04*length1_bar-2*p1[1], -2*p03*p00*length1_bar+2*p03*p02*pow(length1_bar,2)+2*p03*length1_bar*p1[0]-2*p05*p01*length1_bar+2*p05*p04*pow(length1_bar,2)+2*p05*length1_bar*p1[1], 2*p00+2*p02*length2_bar-2*p2[0], 2*p01+2*p04*length2_bar-2*p2[1], 2*p03*p00*length2_bar+2*p03*p02*pow(length2_bar,2)-2*p03*length2_bar*p2[0]+2*p05*p01*length2_bar+2*p05*p04*pow(length2_bar,2)-2*p05*length2_bar*p2[1], 
        2*p00+2*p02*length2_bar-2*p2[0], 2*p01+2*p04*length2_bar-2*p2[1], 2*p03*p00*length2_bar+2*p03*p02*pow(length2_bar,2)-2*p03*length2_bar*p2[0]+2*p05*p01*length2_bar+2*p05*p04*pow(length2_bar,2)-2*p05*length2_bar*p2[1], 2, 0, -2*length1_bar;
    return out;
}

BarPendulumSolver::ddtd1ck_t
BarPendulumSolver::get_ddtd1ck(
    p_t const & p
    ,
    ddtp_t const & ddtp
    ,
    v_t const & v
) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    ddtd1ck_t out;
    out <<
+v(0)*((2)*ddtp(0) + (-2*length1_bar)*ddtp(2))+v(1)*((2)*ddtp(1) + (-2*length1_bar)*ddtp(4))+v(2)*((-2*p03*length1_bar)*ddtp(0) + (-2*p05*length1_bar)*ddtp(1) + (2*p03*pow(length1_bar,2))*ddtp(2) + (-2*p00*length1_bar+2*p02*pow(length1_bar,2)+2*length1_bar*p1[0])*ddtp(3) + (2*p05*pow(length1_bar,2))*ddtp(4) + (-2*p01*length1_bar+2*p04*pow(length1_bar,2)+2*length1_bar*p1[1])*ddtp(5)),
+v(0)*((2)*ddtp(0) + (2*length2_bar)*ddtp(2))+v(1)*((2)*ddtp(1) + (2*length2_bar)*ddtp(4))+v(2)*((2*p03*length2_bar)*ddtp(0) + (2*p05*length2_bar)*ddtp(1) + (2*p03*pow(length2_bar,2))*ddtp(2) + (2*p00*length2_bar+2*p02*pow(length2_bar,2)-2*length2_bar*p2[0])*ddtp(3) + (2*p05*pow(length2_bar,2))*ddtp(4) + (2*p01*length2_bar+2*p04*pow(length2_bar,2)-2*length2_bar*p2[1])*ddtp(5));
    return out;
}

