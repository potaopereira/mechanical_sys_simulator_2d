// ExampleMSSolver
#include "ExampleMS/ExampleMSSolver.hpp"


ExampleMSSolver::c_t
ExampleMSSolver::get_c(p_t const & p, q_t const & q) const {
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
        p00+p02*q00+p03*q01+p01+p04*q00+p05*q01,
        pow(q00,2)*pow(parameter1,-2)+pow(q01,2)*pow(parameter2,-2)-1,
        2*p02*q00*pow(parameter1,-2)+2*p03*q01*pow(parameter2,-2)-2*p04*q00*pow(parameter1,-2)-2*p05*q01*pow(parameter2,-2),
        0;
    return out;
}

ExampleMSSolver::cextra_t
ExampleMSSolver::get_cextra(p_t const & p, q_t const & q) const {
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
        p00+p02*q00+p03*q01+p01+p04*q00+p05*q01,
        pow(q00,2)*pow(parameter1,-2)+pow(q01,2)*pow(parameter2,-2)-1,
        2*p02*q00*pow(parameter1,-2)+2*p03*q01*pow(parameter2,-2)-2*p04*q00*pow(parameter1,-2)-2*p05*q01*pow(parameter2,-2),
        pow(p02,2)+pow(p03,2)-1,
        pow(p04,2)+pow(p05,2)-1,
        p02*p04+p03*p05;
    return out;
}

ExampleMSSolver::d1cextra_t
ExampleMSSolver::get_d1cextra(p_t const & p, q_t const & q) const {
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
        1, 1, q00, q01, q00, q01, 
        0, 0, 0, 0, 0, 0, 
        0, 0, 2*q00*pow(parameter1,-2), 2*q01*pow(parameter2,-2), -2*q00*pow(parameter1,-2), -2*q01*pow(parameter2,-2), 
        0, 0, 2*p02, 2*p03, 0, 0, 
        0, 0, 0, 0, 2*p04, 2*p05, 
        0, 0, p04, p05, p02, p03;
    return out;
}

ExampleMSSolver::d2cextra_t
ExampleMSSolver::get_d2cextra(p_t const & p, q_t const & q) const {
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
        p02+p04, p03+p05, 
        2*q00*pow(parameter1,-2), 2*q01*pow(parameter2,-2), 
        2*p02*pow(parameter1,-2)-2*p04*pow(parameter1,-2), 2*p03*pow(parameter2,-2)-2*p05*pow(parameter2,-2), 
        0, 0, 
        0, 0, 
        0, 0;
    return out;
}

ExampleMSSolver::d1ck_t
ExampleMSSolver::get_d1ck(p_t const & p, q_t const & q) const {
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
        1, 1, p03*q00-p02*q01+p05*q00-p04*q01, 
        0, 0, 0, 
        0, 0, 2*p03*q00*pow(parameter1,-2)-2*p02*q01*pow(parameter2,-2)-2*p05*q00*pow(parameter1,-2)+2*p04*q01*pow(parameter2,-2), 
        1, -1, -p02*q01+p03*q00+p04*q01-p05*q00;
    return out;
}

ExampleMSSolver::d1c_t
ExampleMSSolver::get_d1c(p_t const & p, q_t const & q) const {
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
        1, 1, p03*q00-p02*q01+p05*q00-p04*q01, 0, 0, 0, 
        0, 0, 0, 0, 0, 2*p03*q00*pow(parameter1,-2)-2*p02*q01*pow(parameter2,-2)-2*p05*q00*pow(parameter1,-2)+2*p04*q01*pow(parameter2,-2), 
        0, 0, 2*p03*q00*pow(parameter1,-2)-2*p02*q01*pow(parameter2,-2)-2*p05*q00*pow(parameter1,-2)+2*p04*q01*pow(parameter2,-2), 1, -1, -p02*q01+p03*q00+p04*q01-p05*q00, 
        1, -1, -p02*q01+p03*q00+p04*q01-p05*q00, 0, 0, 0;
    return out;
}

ExampleMSSolver::d2c_t
ExampleMSSolver::get_d2c(p_t const & p, q_t const & q) const {
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
        p02+p04,p03+p05,
        2*q00*pow(parameter1,-2),2*q01*pow(parameter2,-2),
        2*p02*pow(parameter1,-2)-2*p04*pow(parameter1,-2),2*p03*pow(parameter2,-2)-2*p05*pow(parameter2,-2),
        0,0
    ;
    return out;
}

ExampleMSSolver::ddtd1ck_t
ExampleMSSolver::get_ddtd1ck(
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
+v(0)*0+v(1)*0+v(2)*((-q01)*ddtp(2) + (q00)*ddtp(3) + (-q01)*ddtp(4) + (q00)*ddtp(5) + (p03+p05)*ddtq(0) + (-p02-p04)*ddtq(1)),
+v(0)*0+v(1)*0+v(2)*0,
+v(0)*0+v(1)*0+v(2)*((-2*q01*pow(parameter2,-2))*ddtp(2) + (2*q00*pow(parameter1,-2))*ddtp(3) + (2*q01*pow(parameter2,-2))*ddtp(4) + (-2*q00*pow(parameter1,-2))*ddtp(5) + (2*p03*pow(parameter1,-2)-2*p05*pow(parameter1,-2))*ddtq(0) + (-2*p02*pow(parameter2,-2)+2*p04*pow(parameter2,-2))*ddtq(1)),
+v(0)*0+v(1)*0+v(2)*((-q01)*ddtp(2) + (q00)*ddtp(3) + (q01)*ddtp(4) + (-q00)*ddtp(5) + (p03-p05)*ddtq(0) + (-p02+p04)*ddtq(1));
    return out;
}

ExampleMSSolver::ddtd2c_t
ExampleMSSolver::get_ddtd2c(
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
        (1)*ddtp(2) + (1)*ddtp(4), (1)*ddtp(3) + (1)*ddtp(5),
        (2*pow(parameter1,-2))*ddtq(0), (2*pow(parameter2,-2))*ddtq(1),
        (2*pow(parameter1,-2))*ddtp(2) + (-2*pow(parameter1,-2))*ddtp(4), (2*pow(parameter2,-2))*ddtp(3) + (-2*pow(parameter2,-2))*ddtp(5),
        0, 0;
    return out;
}

