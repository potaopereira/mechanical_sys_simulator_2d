#pragma once

// SlidingBallsOnSlopesSymb
#include "SlidingBallsOnSlopes/SlidingBallsOnSlopes.hpp"

// MS2DSolverImpl
#include <mssolver/mssolverimpl.hpp>

typedef MS2DSolverImpl<2, 1, 2, 0> SlidingBallsOnSlopesSolverImpl;

class SlidingBallsOnSlopesSolver:
public SlidingBallsOnSlopesSymb,
public SlidingBallsOnSlopesSolverImpl
{
public:
    SlidingBallsOnSlopesSolver(
    );
    virtual c_t get_c(p_t const & p, q_t const & q) const ;
    virtual d1c_t get_d1c(p_t const & p, q_t const & q) const ;
    virtual cextra_t get_cextra(p_t const & p, q_t const & q) const ;
    virtual d1cextra_t get_d1cextra(p_t const & p, q_t const & q) const ;
    virtual d2cextra_t get_d2cextra(p_t const & p, q_t const & q) const ;
    virtual d1ck_t get_d1ck(p_t const & p, q_t const & q) const ;
    virtual d2c_t get_d2c(p_t const & p, q_t const & q) const ;
    virtual ddtd1ck_t get_ddtd1ck(p_t const & p, ddtp_t const & ddtp, q_t const & q, ddtq_t const & ddtq, v_t const & v) const ;
    virtual ddtd2c_t get_ddtd2c(p_t const & p, ddtp_t const & ddtp, q_t const & q, ddtq_t const & ddtq) const ;
};