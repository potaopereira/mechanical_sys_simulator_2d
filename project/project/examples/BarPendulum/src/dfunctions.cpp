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
        -100*p00+100*p02*length1_bar+150*p01-150*p04*length1_bar-arm_1_squared,
        100*p00+100*p02*length2_bar+150*p01+150*p04*length2_bar-arm_2_squared;
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
        -100*p00+100*p02*length1_bar+150*p01-150*p04*length1_bar-arm_1_squared,
        100*p00+100*p02*length2_bar+150*p01+150*p04*length2_bar-arm_2_squared,
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
        -100, 150, 100*length1_bar, 0, -150*length1_bar, 0, 
        100, 150, 100*length2_bar, 0, 150*length2_bar, 0, 
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
        -100, 150, 100*p03*length1_bar-150*p05*length1_bar, 
        100, 150, 100*p03*length2_bar+150*p05*length2_bar;
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
        -100, 150, 100*p03*length1_bar-150*p05*length1_bar, 100, 150, 100*p03*length2_bar+150*p05*length2_bar, 
        100, 150, 100*p03*length2_bar+150*p05*length2_bar, 0, 0, 0;
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
+v(0)*0+v(1)*0+v(2)*((100*length1_bar)*ddtp(3) + (-150*length1_bar)*ddtp(5)),
+v(0)*0+v(1)*0+v(2)*((100*length2_bar)*ddtp(3) + (150*length2_bar)*ddtp(5));
    return out;
}

