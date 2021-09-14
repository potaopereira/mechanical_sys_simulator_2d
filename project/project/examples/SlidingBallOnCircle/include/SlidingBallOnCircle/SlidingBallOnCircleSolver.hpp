/**
 * @file SlidingBallOnCircleSolver.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Solver for a sliding ball on a circle
 * @details \image html images/ellipse_on_ellipse.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// SlidingBallOnCircleSymb
#include "SlidingBallOnCircle/SlidingBallOnCircle.hpp"

// MS2DSolverImpl
#include <mssolver/mssolverimpl.hpp>

/**
 * @brief A sliding ball on a circle is a mechanical system with 1 rigid body and 1 sliding constraint
 * 
 */
typedef MS2DSolverImpl<1, 0, 1, 0> SlidingBallOnCircleSolverImpl;

/**
 * @brief Class that solves the ODE of a sliding ball on a circle
 * 
 * @details A sliding ball on a circle is a mechanical system with 1 rigid body and 1 sliding constraint
 * \image html images/ellipse_on_ellipse.gif
 */
class SlidingBallOnCircleSolver:
public SlidingBallOnCircleSymb,
public SlidingBallOnCircleSolverImpl
{
public:
    SlidingBallOnCircleSolver(
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