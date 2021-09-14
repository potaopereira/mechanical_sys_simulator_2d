/**
 * @file PointOnSuperEllipseSolver.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Solver for point on super-ellipse mechanical system
 * @details \image html images/point_on_superellipse.gif
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

// PointOnSuperEllipseSymb
#include "PointOnSuperEllipse/PointOnSuperEllipse.hpp"

// MS2DHOLSolverImpl
#include <mssolver/mssolverimpl_specilized.hpp>

/**
 * @brief A point on super-ellipse mechanical system is composed of 1 rigid body and 1 non-sliding constraint
 * 
 */
typedef MS2DHOLSolverImpl<1, 1> PointOnSuperEllipseSolverImpl;

/**
 * @brief Class that solves the ODE of a point on a super-ellipse
 * 
 * @details A point on super-ellipse mechanical system is composed of 1 rigid body and 1 non-sliding constraint
 * \image html images/point_on_superellipse.gif
 */
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