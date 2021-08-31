#pragma once

#include <mssolver/rb2dSymb.hpp>

// IMRB2DPARAM
#include <msinterface/param.hpp>

typedef MRB2DSYMB<1, 0, 1, 1> ExampleMSSymbImpl;

class ExampleMSSymb:
public ExampleMSSymbImpl,
public IMRB2DPARAM
{
public:


    ExampleMSSymb();
    // virtual c_t get_c(p_t const & p, q_t const & q);
    // virtual d1c_t get_d1c(p_t const & p, q_t const & q);
    // virtual cextra_t get_cextra(p_t const & p, q_t const & q);
    // virtual d1cextra_t get_d1cextra(p_t const & p, q_t const & q);
    // virtual d2cextra_t get_d2cextra(p_t const & p, q_t const & q);
    // virtual d1ck_t get_d1ck(p_t const & p, q_t const & q);
    // virtual d2c_t get_d2c(p_t const & p, q_t const & q);
    // virtual ddtd1ck_t get_ddtd1ck(p_t const & p, ddtp_t const & ddtp, q_t const & q, ddtq_t const & ddtq, v_t const & v);
    // virtual ddtd2c_t get_ddtd2c(p_t const & p, ddtp_t const & ddtp, q_t const & q, ddtq_t const & ddtq);

    /**************************************************************************/
    /* IMRB2DPARAM                                                            */
    /**************************************************************************/
    map_t& getFloatParameters() {
        return mParamList;
    };

// do not make this private
    float parameter1;
    float parameter2;
    map_t mParamList;

};