#pragma once

// std::string
#include <string>

// Symbolic
#include <symbolicc++.h>

// std::fstream
#include <fstream>

// tuple_holonomic_constraint_t, tuple_holonomic_constraint_with_contact_t
#include "mssolver/util.hpp"

#include <exception>

// std::list
#include <list>


/**
 * @brief Mechanical system is a (3*N - M1)-th dimensional system
 * 
 * @tparam N number of rigid bodies
 * @tparam M1 number of holonomic constraints
 */
template<int N, int M1>
class MRB2DSYMBHOL
{
public:

    // static const int d = N;
    MRB2DSYMBHOL(
        std::array<sop::tuple_holonomic_constraint_t, M1> const & holonomic_constraints
        ,
        std::string name
    ):
    mName(name)
    ,
    mholonomic_constraints(holonomic_constraints)
    {


    }

    void construct_constraints(

    ){

        for(int i = 0; i < N; ++i){
            for(int j = 0; j < 6; ++j){
                ps[6*i + j] = Symbolic(std::string("p") + std::to_string(i) + std::to_string(j));
            }
        }

        for(int i = 0; i < M1; ++i){
            c[i] = sop::get_holonomic_constraint(mholonomic_constraints[i]);
        }

    }

    void compute_derivatives(){

        sop::getderivative(&c[0], M1, &ps[0], 6*N , &d1c[0][0]);

        for(int i = 0; i < M1; ++i){
            for(int j = 0; j < N; ++j){
                /*
                d/dt p1 = v1
                d/dt p2 = v2

                d/dt r00 = +r01*omega
                d/dt r01 = -r00*omega
                d/dt r10 = +r11*omega
                d/dt r11 = -r10*omega
                 */
                d1ck[i][3*j + 0] = d1c[i][6*j + 0];
                d1ck[i][3*j + 1] = d1c[i][6*j + 1];
                d1ck[i][3*j + 2] = 
                        +ps[6*j + 3]*d1c[i][6*j + 2]
                        -ps[6*j + 2]*d1c[i][6*j + 3]
                        +ps[6*j + 5]*d1c[i][6*j + 4]
                        -ps[6*j + 4]*d1c[i][6*j + 5];
            }
        }

        sop::getderivative(&d1ck[0][0], (M1)*(3*N), &ps[0], 6*N , &d1d1ck[0][0][0]);

        // cextra does not need to consider the M3-non-sliding constraints
        for(int i = 0; i < M1; ++i)
            cextra[i] = c[i];
        for(int j = 0; j < N; ++j){
            Symbolic r00 = ps[6*j + 2];
            Symbolic r01 = ps[6*j + 3];
            Symbolic r10 = ps[6*j + 4];
            Symbolic r11 = ps[6*j + 5];
            cextra[M1 + 3*j + 0] = (r00*r00 + r01*r01 - 1);
            cextra[M1 + 3*j + 1] = (r10*r10 + r11*r11 - 1);
            cextra[M1 + 3*j + 2] = (r00*r10 + r01*r11);
        }
        sop::getderivative(&cextra[0], (M1) + 3*N, &ps[0], 6*N , &d1cextra[0][0]);
    }

    void print_functions(
        std::fstream & f
    )
    {
        construct_constraints();
        compute_derivatives();

        f << "// " << mName << "Solver\n";
        f << "#include \"" << mName << "/" << mName << "Solver.hpp\"\n";
        f << "\n";

        f << mName << "Solver::c_t\n";
        f << mName << "Solver::get_c(p_t const & p) const {\n";

        for(int i = 0; i < 6*N; ++i)
            f << "    double " << ps[i] << " = p(" << i << ");\n";

        f << "    c_t out;\n";
        f << "    out << \n";

        int L = (M1);
        for(int i = 0; i < L; ++i){
            f << "        " << c[i];
            if(i!=L-1){
                f << ",\n";
            }else{
                f << ";\n";
            }
        }
        f << "    return out;\n";
        f << "}\n\n";

        /********************************************************************************/
        /********************************************************************************/

        f << mName << "Solver::cextra_t\n";
        f << mName << "Solver::get_cextra(p_t const & p) const {\n";

        for(int i = 0; i < 6*N; ++i)
            f << "    double " << ps[i] << " = p(" << i << ");\n";

        f << "    cextra_t out;\n";
        f << "    out << \n";

        int K = (M1) + 3*N;
        for(int i = 0; i < K; ++i){
            f << "        " << cextra[i];
            if(i!=K-1){
                f << ",\n";
            }else{
                f << ";\n";
            }
        }
        f << "    return out;\n";
        f << "}\n\n";

        /********************************************************************************/
        /********************************************************************************/

        f << mName << "Solver::d1cextra_t\n";
        f << mName << "Solver::get_d1cextra(p_t const & p) const {\n";

        for(int i = 0; i < 6*N; ++i)
            f << "    double " << ps[i] << " = p(" << i << ");\n";

        f << "    d1cextra_t out;\n";
        f << "    out << \n";

        int N1 = (M1) + 3*N;
        int N2 = 6*N;
        for(int i = 0; i < N1; ++i){
            f << "        ";
            for(int j = 0; j < N2; ++j){
                f << d1cextra[i][j];

                if(j!=(N2-1) || i!=(N1-1)){
                    f << ", ";
                }

            }
            if(i!=(N1-1))
                f << "\n";
            else
                f << ";\n";
        }
        f << "    return out;\n";
        f << "}\n\n";


        /********************************************************************************/
        /********************************************************************************/

        f << mName << "Solver::d1ck_t\n";
        f << mName << "Solver::get_d1ck(p_t const & p) const {\n";

        for(int i = 0; i < 6*N; ++i)
            f << "    double " << ps[i] << " = p(" << i << ");\n";

        f << "    d1ck_t out;\n";
        f << "    out <<\n";

        for(int i = 0; i < M1; ++i){
            f << "        ";
            for(int j = 0; j < 3*N; ++j){
                f << d1ck[i][j];

                if(j!=(3*N-1) || i!=(M1-1))
                    f << ", ";
            }
            if(i!=(M1-1))
                f << "\n";
            else
                f << ";\n";
        }
        f << "    return out;\n";
        f << "}\n\n";

        /********************************************************************************/
        /********************************************************************************/

        f << mName << "Solver::d1c_t\n";
        f << mName << "Solver::get_d1c(p_t const & p) const {\n";

        for(int i = 0; i < 6*N; ++i)
            f << "    double " << ps[i] << " = p(" << i << ");\n";

        f << "    d1c_t out;\n";
        f << "    out <<\n";

        for(int i = 0; i < M1; ++i){
            f << "        ";
            for(int j = 0; j < 6*N; ++j){
                f << d1ck[i][j];

                if(j!=(6*N-1) || i!=(M1-1))
                    f << ", ";
            }
            if(i!=(M1-1))
                f << "\n";
            else
                f << ";\n";
        }
        f << "    return out;\n";
        f << "}\n\n";

        /********************************************************************************/
        /********************************************************************************/

        f << mName << "Solver::ddtd1ck_t\n";
        f << mName << "Solver::get_ddtd1ck(\n";
        f << "    p_t const & p\n";
        f << "    ,\n";
        f << "    ddtp_t const & ddtp\n";
        f << "    ,\n";
        f << "    v_t const & v\n";
        f << ") const {\n";

        for(int i = 0; i < 6*N; ++i)
            f << "    double " << ps[i] << " = p(" << i << ");\n";

        f << "    ddtd1ck_t out;\n";
        f << "    out <<\n";

        for(int i = 0; i < M1; ++i){
            
            for(int j = 0; j < 3*N; ++j){

                std::vector<std::string> v;

                for(int k = 0; k < 6*N; ++k){
                    if(d1d1ck[i][j][k]!=Symbolic(0)){
                        std::stringstream ss;
                        ss << "(" << d1d1ck[i][j][k] << ")" << "*ddtp(" << k << ")";
                        v.push_back(ss.str());
                    }
                }

                if(v.size()==0){
                    f << "+v(" << j << ")*0";
                }
                else{
                    f << "+v(" << j <<")*(";
                    for(std::size_t i = 0; i < v.size(); ++i){
                        f << v[i];
                        if(i!=v.size()-1)
                            f << " + ";
                    }
                    f << ")";
                }

                // end of row
                if(j==3*N-1){
                    if(i!=(M1)-1)
                        f << ",\n";
                    else
                        f << ";\n";
                }

            }
        }

        f << "    return out;\n";

        f << "}\n\n";

    }

private:
    /**
     * @brief position
     * 
     */
    std::string mName;

    std::array<sop::tuple_holonomic_constraint_t, M1> mholonomic_constraints;

    // m constraints
    Symbolic c[M1];
    Symbolic d1c[M1][6*N];
    Symbolic d1ck[M1][3*N];
    Symbolic d1d1ck[M1][3*N][6*N];

    Symbolic cextra[M1 + 3*N];
    Symbolic d1cextra[M1 + 3*N][6*N];

    Symbolic ps[(2+4)*N];

};