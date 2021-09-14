/**
 * @file BallsOnSlopesSolver.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Solver for balls on slope mechanical system
 * @details \image html images/balls_on_slopes.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// BallsOnSlopesSymb
#include "BallsOnSlopes/BallsOnSlopes.hpp"

// MS2DSolverImpl
#include <mssolver/mssolverimpl.hpp>

/**
 * @brief A balls-on-slopes mechanical system is composed of 2 rigid bodies with 2 non sliding constraints
 * 
 */
typedef MS2DSolverImpl<2, 1, 2, 2> BallsOnSlopesSolverImpl;

/**
 * @brief Class that solves the ODE of balls on slopes
 * 
 * @details A balls-on-slopes mechanical system is composed of 2 rigid bodies with 2 non sliding constraints
 * \image html images/balls_on_slopes.gif
 */
class BallsOnSlopesSolver:
public BallsOnSlopesSymb,
public BallsOnSlopesSolverImpl
{
public:
    BallsOnSlopesSolver(
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