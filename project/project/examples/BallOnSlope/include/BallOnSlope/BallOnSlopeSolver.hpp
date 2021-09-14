/**
 * @file BallOnSlopeSolver.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Solver for a ball on a slope mechanical system
 * \image html images/ball_vs_ellipse_on_slope.gif
 * \image html images/ball_vs_flatellipse_on_slope.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// BallOnSlopeSymb
#include "BallOnSlope/BallOnSlope.hpp"

// MS2DSolverImpl
#include <mssolver/mssolverimpl.hpp>

/**
 * @brief A ball on a slope is a mechanical system composed of 1 rigid body and 1 non-sliding constraint
 * 
 */
typedef MS2DSolverImpl<1, 0, 1, 1> BallOnSlopeSolverImpl;

/**
 * @brief Class that solves the ODE of a ball on a slope
 * 
 * @details A ball on a slope is a mechanical system composed of 1 rigid body and 1 non-sliding constraint
 * \image html images/ball_vs_ellipse_on_slope.gif
 * \image html images/ball_vs_flatellipse_on_slope.gif
 */
class BallOnSlopeSolver:
public BallOnSlopeSymb,
public BallOnSlopeSolverImpl
{
public:
    BallOnSlopeSolver(
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