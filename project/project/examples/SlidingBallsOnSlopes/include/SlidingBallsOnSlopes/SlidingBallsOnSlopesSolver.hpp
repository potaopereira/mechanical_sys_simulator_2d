/**
 * @file SlidingBallsOnSlopesSolver.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Solver for sliding balls on slopes mechanical system
 * @details \image html images/ball_vs_ellipse_on_slope.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// SlidingBallsOnSlopesSymb
#include "SlidingBallsOnSlopes/SlidingBallsOnSlopes.hpp"

// MS2DSolverImpl
#include <mssolver/mssolverimpl.hpp>

/**
 * @brief A sliding-balls-on-slopes mechanical system is composed of 2 rigid bodies with 2 sliding constraints
 * 
 */
typedef MS2DSolverImpl<2, 1, 2, 0> SlidingBallsOnSlopesSolverImpl;

/**
 * @brief Class that solves the ODE of a sliding balls on slopes
 * 
 * @details A sliding-balls-on-slopes mechanical system is composed of 2 rigid bodies with 2 sliding constraints
 * 
 */
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