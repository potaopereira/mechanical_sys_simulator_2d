/**
 * @file exceptions.hpp
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include <exception>

/**
 * @brief Exception to be used if predicting the motion of the mechanical system fails at some point
 * 
 */
class SolverException:
virtual public std::exception {
public:

    /**
     * @brief Construct a solver exception
     * 
     */
    SolverException():
    std::exception(),
    error_message("Solver exception.")
    {

    }

    /**
     * @brief Destroy the Solver Exception object
     * 
     */
    virtual ~SolverException() throw () {}


    /**
     * @brief Provide exception description
     * 
     * @return const char* Pointer to error message
     */
    virtual
    const char* what() const throw () {
       return error_message.c_str();
    }

private:
    std::string error_message;
};

/**
 * @brief Exception throwned when the matrix d2c(p,q) in R^{2*M2, M1+3*M2+M} is not full rank
 * 
 */
class RankLossException:
virtual public SolverException {
public:

    /**
     * @brief Construct a new Rank Loss Exception object
     * 
     */
    RankLossException():
    SolverException(),
    error_message("d2c(p,q) in R^{2*M2, M1+3*M2+M} is not full rank.")
    {

    }

    /**
     * @brief Destroy the Rank Loss Exception object
     * 
     */
    virtual ~RankLossException() throw () {}


    /**
     * @brief Provide exception description
     * 
     * @return const char* Pointer to error message
     */
    virtual
    const char* what() const throw () {
       return error_message.c_str();
    }

private:
    std::string error_message;
};