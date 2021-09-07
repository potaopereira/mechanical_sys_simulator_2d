#pragma once

// PointOnSuperEllipseSymb
#include "PointOnSuperEllipse/PointOnSuperEllipse.hpp"

// MS2DHOLSolverImpl
#include <mssolver/mssolverimpl_specilized.hpp>

typedef MS2DHOLSolverImpl<1, 1> PointOnSuperEllipseSolverImpl;

class PointOnSuperEllipseSolver:
public PointOnSuperEllipseSymb,
public PointOnSuperEllipseSolverImpl
{
public:
    PointOnSuperEllipseSolver(
    );
    virtual c_t get_c(p_t const & p) const ;
    virtual d1c_t get_d1c(p_t const & p) const ;
    virtual cextra_t get_cextra(p_t const & p) const ;
    virtual d1cextra_t get_d1cextra(p_t const & p) const ;
    virtual d1ck_t get_d1ck(p_t const & p) const ;
    virtual ddtd1ck_t get_ddtd1ck(p_t const & p, ddtp_t const & ddtp, v_t const & v) const ;
};