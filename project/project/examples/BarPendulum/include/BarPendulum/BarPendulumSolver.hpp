/**
 * @file BarPendulumSolver.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Solver for bar pendulum mechanical system
 * @details \image html images/bar_pendulum.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// BarPendulumSymb
#include "BarPendulum/BarPendulum.hpp"

// MS2DHOLSolverImpl
#include <mssolver/mssolverimpl_specilized.hpp>

/**
 * @brief A bar pendulum mechanical system is composed of 1 rigid body and two holonomic constraints
 * 
 */
typedef MS2DHOLSolverImpl<1, 2> BarPendulumSolverImpl;

/**
 * @brief Class that solves the ODE of a bar pendulum
 * 
 * @details A bar pendulum mechanical system is composed of 1 rigid body and two holonomic constraints
 * \image html images/bar_pendulum.gif
 */
class BarPendulumSolver:
public BarPendulumSymb,
public BarPendulumSolverImpl
{
public:
    BarPendulumSolver(
    );
    virtual c_t get_c(p_t const & p) const ;
    virtual d1c_t get_d1c(p_t const & p) const ;
    virtual cextra_t get_cextra(p_t const & p) const ;
    virtual d1cextra_t get_d1cextra(p_t const & p) const ;
    virtual d1ck_t get_d1ck(p_t const & p) const ;
    virtual ddtd1ck_t get_ddtd1ck(p_t const & p, ddtp_t const & ddtp, v_t const & v) const ;
};