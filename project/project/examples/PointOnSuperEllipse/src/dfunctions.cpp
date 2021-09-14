// PointOnSuperEllipseSolver
#include "PointOnSuperEllipse/PointOnSuperEllipseSolver.hpp"

PointOnSuperEllipseSolver::c_t
PointOnSuperEllipseSolver::get_c(p_t const & p) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    c_t out;
    out << 
        pow(p00,outside_alpha)*pow(outside_rinv,outside_alpha)+pow(p01,outside_alpha)*pow(outside_rinv,outside_alpha)-1;
    return out;
}

PointOnSuperEllipseSolver::cextra_t
PointOnSuperEllipseSolver::get_cextra(p_t const & p) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    cextra_t out;
    out << 
        pow(p00,outside_alpha)*pow(outside_rinv,outside_alpha)+pow(p01,outside_alpha)*pow(outside_rinv,outside_alpha)-1,
        pow(p02,2)+pow(p03,2)-1,
        pow(p04,2)+pow(p05,2)-1,
        p02*p04+p03*p05;
    return out;
}

PointOnSuperEllipseSolver::d1cextra_t
PointOnSuperEllipseSolver::get_d1cextra(p_t const & p) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    d1cextra_t out;
    out << 
        outside_alpha*pow(p00,outside_alpha-1)*pow(outside_rinv,outside_alpha), outside_alpha*pow(p01,outside_alpha-1)*pow(outside_rinv,outside_alpha), 0, 0, 0, 0, 
        0, 0, 2*p02, 2*p03, 0, 0, 
        0, 0, 0, 0, 2*p04, 2*p05, 
        0, 0, p04, p05, p02, p03;
    return out;
}

PointOnSuperEllipseSolver::d1ck_t
PointOnSuperEllipseSolver::get_d1ck(p_t const & p) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    d1ck_t out;
    out <<
        outside_alpha*pow(p00,outside_alpha-1)*pow(outside_rinv,outside_alpha), outside_alpha*pow(p01,outside_alpha-1)*pow(outside_rinv,outside_alpha), 0;
    return out;
}

PointOnSuperEllipseSolver::d1c_t
PointOnSuperEllipseSolver::get_d1c(p_t const & p) const {
    double p00 = p(0);
    double p01 = p(1);
    double p02 = p(2);
    double p03 = p(3);
    double p04 = p(4);
    double p05 = p(5);
    d1c_t out;
    out <<
        outside_alpha*pow(p00,outside_alpha-1)*pow(outside_rinv,outside_alpha), outside_alpha*pow(p01,outside_alpha-1)*pow(outside_rinv,outside_alpha), 0, 0, 0, 0;
    return out;
}

PointOnSuperEllipseSolver::ddtd1ck_t
PointOnSuperEllipseSolver::get_ddtd1ck(
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
+v(0)*((pow(outside_alpha,2)*pow(p00,outside_alpha-2)*pow(outside_rinv,outside_alpha)-outside_alpha*pow(p00,outside_alpha-2)*pow(outside_rinv,outside_alpha))*ddtp(0))+v(1)*((pow(outside_alpha,2)*pow(p01,outside_alpha-2)*pow(outside_rinv,outside_alpha)-outside_alpha*pow(p01,outside_alpha-2)*pow(outside_rinv,outside_alpha))*ddtp(1))+v(2)*0;
    return out;
}

