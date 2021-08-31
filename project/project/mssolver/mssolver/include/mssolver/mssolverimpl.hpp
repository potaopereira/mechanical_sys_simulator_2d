#pragma once

// std::array
#include  <array>

#include <string>

#include <Eigen/Dense>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>


#include <gsl/gsl_multimin.h>

#include "mssolver/util.hpp"

// IMS2DSolver
#include "msinterface/solver.hpp"

/**
 * @brief system is a (3*N + 2*M2 - (M1 + 3*M2))-th dimensional system
 * 
 * 3*N + M1 - M2
 * 
 * @tparam N number of rigid bodies
 * @tparam m number of constraints
 * @tparam p number of auxiliary variables
 */
template<int N, int M1, int M2, int M3>
class MS2DSolverImpl:
public IMS2DSolver
{
public:

    // static const int d = N;
    MS2DSolverImpl(
        IMSConstraints<N, M1, M2, M3>* ptr
        // ,
        // std::array<std::pair<, N> inertias
    )
    {
        mListWithContact = ptr->getListOrderedWithContact();

        for(int i = 0; i < N; ++i){
            Eigen::Matrix<double, 3, 3> m;
            m << 
                1, 0, 0,
                0, 1, 0,
                0, 0, 1;
            mIinv.block(3*i, 3*i, 3, 3) = m;
            mI.block(3*i, 3*i, 3, 3) = m;
        }

    }

    Eigen::Matrix<double, 2+4, 1> VelocityddtP(
        rbp_t const & p
        ,
        rbv_t const & v
    ){
        // // without stability
        // rbv_t ddtp;
        // ddtp(0) = v(0);
        // ddtp(1) = v(1);
        // ddtp(2) = +v(2)*p(3);
        // ddtp(3) = -v(2)*p(2);
        // ddtp(4) = +v(2)*p(5);
        // ddtp(5) = -v(2)*p(4);

        // with stability
        Eigen::Matrix<double, 2+4, 1> ddtp;
        ddtp(0) = v(0);
        ddtp(1) = v(1);

        // these numbers will be close to zero
        double e1 = p(2)*p(2) + p(4)*p(4) - 1;
        double e2 = p(3)*p(3) + p(5)*p(5) - 1;
        double e3 = p(2)*p(3) + p(4)*p(5);

        ddtp(2) = +v(2)*p(3) - p(2)*e1           - p(3)*e3;
        ddtp(3) = -v(2)*p(2)           - p(3)*e2 - p(2)*e3;
        ddtp(4) = +v(2)*p(5) - p(4)*e1           - p(5)*e3;
        ddtp(5) = -v(2)*p(4)           - p(5)*e2 - p(4)*e3;
        return ddtp;
    }


    virtual int dim() const { return N; }
    virtual int dimSolution() const {
        return 
            6*N  +  // positions
            3*N  +  // velocities
            3*N  +  // forces
            2*M2    // contact points
        ;
    }
    virtual int dimRigidBodies() const {return N;}
    virtual int dimConstraints() const {return M1+M2+M3;}
    
    virtual int dimConstr() const {return M1+3*M2+M3;}
    virtual int dimM1() const {return M1;}
    virtual int dimM2() const {return M2;}


    // virtual
    // void show(
    //     double *solution
    // ){

    //     c_t _c = get_c(
    //         p_t(&solution[0])
    //         ,
    //         q_t(&solution[6*N])
    //         );

    //     std::array<std::vector<Eigen::Matrix<double, 2, 1>>, N> inertial;
    //     std::array<std::vector<Eigen::Matrix<double, 2, 1>>, N> body;
    //     for(int i = 0; i < N; ++i){
    //         inertial[i] = std::vector<Eigen::Matrix<double, 2, 1>>({});
    //         body[i] = std::vector<Eigen::Matrix<double, 2, 1>>({});
    //     }

    //     for(int i = 0; i < M2; ++i){
    //         int k = mholonomic_constraint_with_contact[i].rigid_body;
    //         body[k] = std::vector<Eigen::Matrix<double, 2, 1>>(
    //             {
    //                 Eigen::Matrix<double, 2, 1>(&solution[6*N])
    //             }
    //         );
    //     }

    //     mMSView->setPoseAndVectors(
    //         p_t(&solution[0]),
    //         100*v_t(&solution[6*N+2*M2]),
    //         f_t(&solution[6*N+2*M2+3*N]),
    //         inertial,
    //         body
    //     );
    // }

    virtual
    rbp_t getPositionPlot(
        double* solution,
        int k // rigid body number
    ) const {
        return rbp_t(&solution[6*k + 0]);
    }

    virtual
    rbv_t getVelocityPlot(
        double* solution,
        int k // rigid body number
    ) const {
        return rbv_t(&solution[6*N + 2*M2 + 3*k]);
    }

    virtual
    rbf_t getForcePlot(
        double* solution,
        int k // rigid body number
    ) const {
        return rbf_t(&solution[6*N + 2*M2 + 3*N + 3*k]);
    }


    virtual
    std::vector<IMS2DSolver::v_t> getInertialVectorsPlot(
        double* solution,
        int k // rigid body number
    ) const {
        return std::vector<IMS2DSolver::v_t>({});
    }

    virtual
    std::vector<IMS2DSolver::v_t> getBodyVectorsPlot(
        double* solution,
        int k // rigid body number
    ) const {

        return std::vector<IMS2DSolver::v_t>(
            {
                IMS2DSolver::v_t(&solution[6*N + 2*k])
            }
        );
        // for (std::list<int>::iterator it = mListWithContact.begin(); it!=mListWithContact.end(); ++it){
        //     if(*it==k){

        //     }
        // }

        
        // std::list<int>::iterator j_ptr = std::find(std::begin(mListWithContact), std::end(mListWithContact), k);
        // if(j_ptr==std::end(list)){
        //     return std::vector<rbv_t>({});
        // }else{
        //     int j = *j_ptr;
        //     return std::vector<rbv_t>(
        //         {
        //             rbv_t(&solution[6*N + 2*j])
        //         }
        //     );
        // }
    }


    virtual
    std::array<double, 3> getPosition(
        double* solution,
        int k // rigid body number
    ) const {
        return std::array<double, 3>(
            {{
                solution[6*k + 0],
                solution[6*k + 1],
                180/M_PI*std::atan2(solution[6*k + 4], solution[6*k + 2])
            }}
        );
    }

    virtual
    std::array<double, 3> getVelocity(
        double* solution,
        int k // rigid body number
    ) const {
        return std::array<double, 3>(
            {{
                solution[6*N + 2*M2 + 3*k + 0],
                solution[6*N + 2*M2 + 3*k + 1],
                solution[6*N + 2*M2 + 3*k + 2]
            }}
        );
    }

    virtual
    std::array<double, 3> getForce(
        double* solution,
        int k // rigid body number
    ) const {
        return std::array<double, 3>(
            {{
                solution[6*N + 2*M2 + 3*N + 3*k + 0],
                solution[6*N + 2*M2 + 3*N + 3*k + 1],
                solution[6*N + 2*M2 + 3*N + 3*k + 2]
            }}
        );
    }

    virtual
    std::array<double, 2> getPositionOfPoint(
        double* solution,
        int k, // rigid body number
        std::array<double, 2> relative_position
    ) const {
        double p[2] = {solution[6*k + 0], solution[6*k + 1]};
        double r[2][2] = {
                {solution[6*k + 2], solution[6*k + 3]},
                {solution[6*k + 4], solution[6*k + 5]}
            };
        return std::array<double, 2>(
            {{
                p[0] + r[0][0]*relative_position[0] + r[0][1]*relative_position[1],
                p[1] + r[1][0]*relative_position[0] + r[1][1]*relative_position[1]
            }}
        );
    }

    virtual
    std::array<double, 2> getVelocityOfPoint(
        double* solution,
        int k, // rigid body number
        std::array<double, 2> relative_position
    ) const {
        double v[2] = {solution[6*N + 2*M2 + 3*k + 0], solution[6*N + 2*M2 + 3*k + 1]};
        double omega = solution[6*N + 2*M2 + 3*k + 2];
        double r[2][2] = {
                {solution[6*k + 2], solution[6*k + 3]},
                {solution[6*k + 4], solution[6*k + 5]}
            };
        return std::array<double, 2>(
            {{
                v[0] + omega*(-r[0][1]*relative_position[0] + r[0][0]*relative_position[1]),
                v[1] + omega*(-r[1][1]*relative_position[0] + r[1][0]*relative_position[1])
            }}
        );
    }

    typedef Eigen::Matrix<double, 6*N, 1> p_t;
    typedef Eigen::Matrix<double, 6*N, 1> ddtp_t;

    typedef Eigen::Matrix<double, 2*M2, 1> q_t;
    typedef Eigen::Matrix<double, 2*M2, 1> ddtq_t;

    typedef Eigen::Matrix<double, 3*N, 1> v_t;
    typedef Eigen::Matrix<double, 3*N, 1> ddtv_t;

    typedef Eigen::Matrix<double, 6*N + 2*M2 + 3*N, 1> state_t;
    typedef Eigen::Matrix<double, 6*N + 2*M2 + 3*N, 1> ddtstate_t;

    typedef Eigen::Matrix<double, 3*N, 1> f_t;

    typedef Eigen::Matrix<double, M1+3*M2+M3, 3*N> dkc_t;



    typedef Eigen::Matrix<double, M1+3*M2+M3, 1> c_t;

    typedef Eigen::Matrix<double, M1+3*M2 + 3*N, 1> cextra_t;
    typedef Eigen::Matrix<double, M1+3*M2 + 3*N, 6*N> d1cextra_t;
    typedef Eigen::Matrix<double, M1+3*M2 + 3*N, 2*M2> d2cextra_t;

    typedef Eigen::Matrix<double, M1+3*M2+M3, 6*N> d1c_t;
    typedef Eigen::Matrix<double, M1+3*M2+M3, 3*N> d1ck_t;
    typedef Eigen::Matrix<double, M1+3*M2+M3, 2*M2> d2c_t;
    typedef Eigen::Matrix<double, M1+3*M2+M3, 1> ddtd1ck_t;
    typedef Eigen::Matrix<double, M1+3*M2+M3, 2*M2> ddtd2c_t;

    p_t getP(state_t const & x) {
        return x.segment(0, 6*N);
    }

    v_t getV(state_t const & x) {
        return x.segment(6*N, 3*N);
    }

    ddtp_t
    ddtP(
        p_t const & p
        ,
        v_t const & v
    ){

        ddtp_t ddtp = ddtp_t::Zero();

        for(int i = 0; i < N; ++i){
            int k = 6*i;

            Eigen::Matrix<double, 6, 1> pi = p.segment(k, 6);
            Eigen::Matrix<double, 3, 1> vi = v.segment(k, 3);

            ddtp.segment(k, 6) = VelocityddtP(pi, vi);
        }
        return ddtp;
    }

    ddtp_t
    ddtP(
        state_t const & x
    ){
        return ddtP(getP(x), getV(x));
    }

    ddtv_t
    getInternalForces(
        p_t const & p,
        v_t const & v,
        f_t const & f
    ){
        ddtv_t ddtv;

        // acceleration no constraints
        ddtv_t free = mIinv*f;

        // Eigen::Matrix<double, M1+3*M2+M3, 3*N> dkc = getdkc(p);
        // Eigen::Matrix<double, M1+3*M2+M3, 1> d2c = getd2c(p, v);

        // Eigen::Matrix<double, M1+3*M2+M3, M1+3*M2+M3> to_invert;
        // to_invert = dkc*mIinv*dkc.transpose();

        // ddtv = -dkc.transpose()*to_invert.llt().solve(
        //     dkc*free
        //     +
        //     d2c
        // );

        return ddtv;
    }

    ddtv_t
    getInternalForces(
        state_t const & x
    ){
        return getInternalForces(getP(x), getV(x), getForce(x));
    }

    ddtv_t
    ddtV(
        p_t const & p,
        v_t const & v,
        f_t const & f
    ){

        q_t q;

        ddtv_t ddtv;

        // acceleration no constraints
        ddtv_t free = mIinv*f;


        d1ck_t A = get_d1ck(p, q);
        d2c_t B = get_d2c(p, q);
        Eigen::Matrix<double, 2*M2, M1+3*M2+M3> BT = B.transpose();
        Eigen::Matrix<double, M1+M2+M3, M1+3*M2+M3> Bker = B.transpose().fullPivLu().kernel();
        

        /* 
            N v = 0
            where N = (I - B ( B^T B )^-1 B^T) A
            where B = d2c and A = d1c
        */
        // Eigen::Matrix<double, 2*M2, 2*M2> inv = (B.transpose()*B).inverse();
        // Eigen::Matrix<double, m, m> PI_B = Eigen::Matrix<double, m, m>::Identity() - B*inv*BT;
        // Eigen::Matrix<double, m, 3*N> NS = PI_B*A;
        // ddtq_t ddtq = - inv*BT*A*v;

        ddtq_t ddtq = - (B.transpose()*B).llt().solve(BT*A*v);

        ddtp_t p_dot;

        // ddt(A) v = (dP(A) Pdot + dQ(A) Qdto) v
        Eigen::Matrix<double, M1+3*M2+M3, 1> dA = get_ddtd1ck(p, p_dot, q, ddtq, v);

        // ddt [B]   = dP(B) Pdot + dQ(B) Qdot
        Eigen::Matrix<double, M1+3*M2+M3, 2*M2> dB = get_ddtd2c(p, p_dot, q, ddtq);


        /*
            d/dt [(I - B ( B^T B )^-1 B^T) A] v
            =
            d/dt [(I - B ( B^T B )^-1 B^T)] A v
            +
            (I - B ( B^T B )^-1 B^T) d/dt[A] v
            =
            (
                - dB ( B^T B )^-1 B^T
                - B ( B^T B )^-1 dB^T
                + B ( B^T B )^-1 (dB^T B + B^T dB) ( B^T B )^-1 B^T
            ) A v
            +
            (I - B ( B^T B )^-1 B^T) dA v
            =
            (
                - dB ( B^T B )^-1 B^T
                - B ( B^T B )^-1 dB^T
                + B ( B^T B )^-1 dB^T B ( B^T B )^-1 B^T
                + B ( B^T B )^-1 B^T dB ( B^T B )^-1 B^T
            ) A v
            +
            (I - B ( B^T B )^-1 B^T) dA v
            =
            (
                (B ( B^T B )^-1 B^T - I) dB ( B^T B )^-1 B^T
                +
                B ( B^T B )^-1 dB^T ( B ( B^T B )^-1 B^T - I)
            ) A v
            +
            (I - B ( B^T B )^-1 B^T) dA v
            =
            (
                - PI_B dB ( B^T B )^-1 B^T
                - B ( B^T B )^-1 dB^T PI_B
            ) A v
            +
            (I - B ( B^T B )^-1 B^T) dA v

            but PI_B A v = 0, so
   
            PI_B (- dB ( B^T B )^-1 B^T A v + dA v)
        */

        // Eigen::Matrix<double, m, 1> Av = A*v;
        // Eigen::Matrix<double, m, 1> virtual_acceleration = PI_B*(dA - dB*inv*BT*Av);

        Eigen::Matrix<double, M1+M2+M3, 3*N> NSS = Bker*A;

        Eigen::Matrix<double, M1+M2+M3, M1+M2+M3> to_invert;
        to_invert = NSS*mIinv*NSS.transpose();

        ddtv =
        free -
        mIinv*NSS.transpose()*to_invert.llt().solve(
            NSS*free
            +
            Bker*(dA + dB*ddtq)
            // Bker*(dA - dB*inv*BT*Av)
        );

        return ddtv;
    }


    ddtv_t
    ddtV(
        state_t const & x
    ){
        return ddtV(getP(x), getV(x), getForce(x));
    }

    virtual
    f_t
    getForce(
        p_t const & p
        ,
        v_t const & v
    ){
        f_t f = f_t::Zero();
        f << 0, -1, 0;
        return f;
    }

    virtual
    f_t getForce(
        state_t const & state
    ){
        return getForce(getP(state), getV(state));
    }

    v_t getProjectedVelocity(
        p_t const & p
        ,
        q_t const & q
        ,
        v_t const & v
    ){
        d1ck_t A = get_d1ck(p, q);
        d2c_t B = get_d2c(p, q);
        Eigen::Matrix<double, M1+M2+M3, M1+3*M2+M3> Bker = B.transpose().fullPivLu().kernel().transpose();
        Eigen::Matrix<double, M1+M2+M3, 3*N> NS = Bker*A;
        Eigen::Matrix<double, M1+M2+M3, M1+M2+M3> to_invert = NS*NS.transpose();
        v_t v_projected = v - NS.transpose()*to_invert.llt().solve(NS*v);
        return v_projected;
    }

    ddtstate_t getD(
        state_t const & state
    ){
        // ddtstate_t ddtstate;
        // ddtstate.segment(0  , 6*N) = ddtP(state);
        // ddtstate.segment(6*N, 3*N) = ddtV(state);
        // return ddtstate;

        p_t p = state.segment(0, 6*N);
        q_t q = state.segment(6*N, 2*M2);
        v_t v = state.segment(6*N + 2*M2, 3*N);

        ddtv_t ddtv;

        f_t f = getForce(state);
        // acceleration no constraints
        ddtv_t free = mIinv*f;


        d1ck_t A = get_d1ck(p, q);
        d2c_t B = get_d2c(p, q);
        Eigen::Matrix<double, 2*M2, M1+3*M2+M3> BT = B.transpose();
        // Eigen::Matrix<double, m, m - Q> Bker_aux = BT.fullPivLu().kernel();
        // Eigen::Matrix<double, m, m - Q> Bker_aux = Eigen::Matrix<double, m, m - Q>::Zero();
        // std::cout << B << std::endl;
        // std::cout << BT.fullPivLu().kernel() << std::endl;
        Eigen::Matrix<double, M1+M2+M3, M1+3*M2+M3> Bker = BT.fullPivLu().kernel().transpose();
        // std::cout << BT << std::endl;
        // std::cout << BT.fullPivLu().kernel() << std::endl;
        // std::cout << BT.fullPivLu().kernel().transpose()*A << std::endl;
        // std::cout << Bker_aux << std::endl;
        // std::cout << Bker << std::endl;

        /* 
            N v = 0
            where N = (I - B ( B^T B )^-1 B^T) A
            where B = d2c and A = d1c
        */
        // Eigen::Matrix<double, Q, Q> inv = (B.transpose()*B).inverse();
        // Eigen::Matrix<double, m, m> PI_B = Eigen::Matrix<double, m, m>::Identity() - B*inv*BT;
        // Eigen::Matrix<double, m, 3*N> NS = PI_B*A;
        // ddtq_t ddtq = - inv*BT*A*v;

        Eigen::Matrix<double, M1+M2+M3, 3*N> NSS = Bker*A;


        // add stability terms
        // for stability terms, we must consider the non holonomic constraints
        c_t _c = get_c(p, q);
        // _c should be zero, in which case, stability = 0
        Eigen::Matrix<double, M1+3*M2+M3, 3*N + 2*M2> K;
        K << A, B;
        Eigen::Matrix<double, 3*N + 2*M2, 1> stability = -K.transpose()*(K*K.transpose()).llt().solve(_c);
        v_t v_stability = stability.segment(0, 3*N);
        ddtq_t ddtq_stability = stability.segment(3*N, 2*M2);

        // ddtp_t p_dot = ddtP(p, v);
        // ddtq_t ddtq = - (B.transpose()*B).llt().solve(BT*A*v);

        ddtp_t p_dot = ddtP(p, v + v_stability);
        ddtq_t ddtq = - (B.transpose()*B).llt().solve(BT*A*v) + ddtq_stability;


        // ddt(A) v = (dP(A) Pdot + dQ(A) Qdto) v
        Eigen::Matrix<double, M1+3*M2+M3, 1> dA = get_ddtd1ck(p, p_dot, q, ddtq, v);

        // ddt [B]   = dP(B) Pdot + dQ(B) Qdot
        Eigen::Matrix<double, M1+3*M2+M3, 2*M2> dB = get_ddtd2c(p, p_dot, q, ddtq);


        /*
            d/dt [(I - B ( B^T B )^-1 B^T) A] v
            =
            d/dt [(I - B ( B^T B )^-1 B^T)] A v
            +
            (I - B ( B^T B )^-1 B^T) d/dt[A] v
            =
            (
                - dB ( B^T B )^-1 B^T
                - B ( B^T B )^-1 dB^T
                + B ( B^T B )^-1 (dB^T B + B^T dB) ( B^T B )^-1 B^T
            ) A v
            +
            (I - B ( B^T B )^-1 B^T) dA v
            =
            (
                - dB ( B^T B )^-1 B^T
                - B ( B^T B )^-1 dB^T
                + B ( B^T B )^-1 dB^T B ( B^T B )^-1 B^T
                + B ( B^T B )^-1 B^T dB ( B^T B )^-1 B^T
            ) A v
            +
            (I - B ( B^T B )^-1 B^T) dA v
            =
            (
                (B ( B^T B )^-1 B^T - I) dB ( B^T B )^-1 B^T
                +
                B ( B^T B )^-1 dB^T ( B ( B^T B )^-1 B^T - I)
            ) A v
            +
            (I - B ( B^T B )^-1 B^T) dA v
            =
            (
                - PI_B dB ( B^T B )^-1 B^T
                - B ( B^T B )^-1 dB^T PI_B
            ) A v
            +
            (I - B ( B^T B )^-1 B^T) dA v

            but PI_B A v = 0, so
   
            PI_B (- dB ( B^T B )^-1 B^T A v + dA v)
        */

        // Eigen::Matrix<double, m, 1> Av = A*v;
        // Eigen::Matrix<double, m, 1> virtual_acceleration = PI_B*(dA - dB*inv*BT*Av);

        Eigen::Matrix<double, M1+M2+M3, M1+M2+M3> to_invert;
        to_invert = NSS*mIinv*NSS.transpose();

        ddtv =
        free -
        mIinv*NSS.transpose()*to_invert.llt().solve(
            NSS*(free + v)
            +
            Bker*(dA + dB*ddtq)
            // Bker*(dA - dB*inv*BT*Av)
        );

        Eigen::Matrix<double, M1+M2+M3, 1> error = NSS*v;

        ddtstate_t ddtstate;
        ddtstate.segment(0    , 6*N) = p_dot;
        ddtstate.segment(6*N  , 2*M2  ) = ddtq;
        ddtstate.segment(6*N+2*M2, 3*N) = ddtv;
        return ddtstate;

    }


    int
    functionGSL(
        double t,
        const double y[],
        double ddty[],
        void * params
    )
    {

        state_t state(y);
        ddtstate_t ddtstate = getD(state);
        ddty = ddtstate.data();

        return GSL_SUCCESS;
    }


    virtual c_t get_c(p_t const & p, q_t const & q) const = 0;
    virtual cextra_t get_cextra(p_t const & p, q_t const & q) const = 0;
    virtual d1cextra_t get_d1cextra(p_t const & p, q_t const & q) const = 0;
    virtual d2cextra_t get_d2cextra(p_t const & p, q_t const & q) const = 0;
    virtual d1c_t get_d1c(p_t const & p, q_t const & q) const = 0;
    virtual d1ck_t get_d1ck(p_t const & p, q_t const & q) const = 0;
    virtual d2c_t get_d2c(p_t const & p, q_t const & q) const = 0;
    virtual ddtd1ck_t get_ddtd1ck(p_t const & p, ddtp_t const & ddtp, q_t const & q, ddtq_t const & ddtq, v_t const & v) const = 0;
    virtual ddtd2c_t get_ddtd2c(p_t const & p, ddtp_t const & ddtp, q_t const & q, ddtq_t const & ddtq) const = 0;
    
    // virtual Eigen::Matrix<double, M1+3*M2+M3, 3*N> getdkc(p_t const & p) {};

    // virtual
    // Eigen::Matrix<double, M1+3*M2+M3, 3*N>
    // getdkcS(
    //     p_t const & p
    // ){

    //     Eigen::Matrix<double, M1+3*M2+M3, 3*N> out;

    //     Equations eq;
    //     for(int i = 0; i < N; ++i){
    //         Symbolic s = mRB[i].getP().getS();
    //         for(int j = 0; j < 6; ++j){
    //             eq = (eq, s(j) == p(6*i + j) );
    //         }
    //     }

    //     for(int i = 0; i < M1+3*M2+M3; ++i){
    //         for(int j = 0; j < 3*N; ++j){ 
    //             Symbolic s = d1ck[i][j];
    //             out(i, j) = s[eq];
    //         }

    //     }
    //     return out;

    // }

    // virtual
    // Eigen::Matrix<double, M1+3*M2+M3, 1>
    // getd2c(
    //     p_t const & p
    //     ,
    //     v_t const & v
    // ){}

    // virtual
    // Eigen::Matrix<double, M1+3*M2+M3, 1>
    // getd2cS(
    //     p_t const & p
    //     ,
    //     v_t const & v
    // ){

    //     Eigen::Matrix<double, M1+3*M2+M3, 1> out = Eigen::Matrix<double, M1+3*M2+M3, 1>::Zero();

    //     ddtp_t ddtp = ddtP(p, v);

    //     Equations eq;
    //     for(int i = 0; i < N; ++i){
    //         Symbolic s = mRB[i].getP().getS();
    //         for(int j = 0; j < 6; ++j){
    //             eq = (eq, s(j) == p(6*i + j) );
    //         }
    //     }

    //     for(int i = 0; i < M1+3*M2+M3; ++i){
    //         for(int j = 0; j < 3*N; ++j){
    //             for(int k = 0; k < 6*N; ++k){
    //                 Symbolic s = d1d1ck[i][j][k];
    //                 double tmp = s[eq];
    //                 out(i) += tmp*ddtp(k)*v(j);
    //             }
    //         }

    //     }
    //     return out;

    // }

    static
    double
    my_f(
        const gsl_vector *v
        ,
        void *params
    )
    {
        MS2DSolverImpl<N, M1, M2, M3> *p = (MS2DSolverImpl<N, M1, M2, M3> *)params;

        cextra_t _c = p->get_cextra(
            p_t(gsl_vector_const_ptr(v, 0))
            ,
            q_t(gsl_vector_const_ptr(v, 6*N))
        );

        return pow(_c.norm(), 2);

        // Equations eq;
        // for(int i = 0; i < N; ++i){
        //     Symbolic s = p->mRB[i].getP().getS();
        //     for(int j = 0; j < 6; ++j){
        //         eq = (eq, s(j) == gsl_vector_get(v, i*6 + j));
        //     }
        // }

        // for(int i = 0; i < 2*M2; ++i){
        //     eq = (eq, p->qs[i] == gsl_vector_get(v, 6*N + i));
        // }

        // double out = 0;
        // for(int i = 0; i < M1+3*M2; ++i){
        //     Symbolic s = p->c[i];
        //     double e = s[eq];
        //     out += e*e/2;
        // }

        // // for each body, rotation matrix constaints
        // for(int i = 0; i < N; ++i){
        //     double r00 = gsl_vector_get(v, i*6 + 2);
        //     double r01 = gsl_vector_get(v, i*6 + 3);
        //     double r10 = gsl_vector_get(v, i*6 + 4);
        //     double r11 = gsl_vector_get(v, i*6 + 5);

        //     double c1 = (r00*r00 + r01*r01 - 1);
        //     double c2 = (r10*r10 + r11*r11 - 1);
        //     double c3 = (r00*r10 + r01*r11);

        //     out += (c1*c1 + c2*c2 + c3*c3)/2;
        // }
        // return out;

    }

    /* The gradient of f, df = (df/dx, df/dy). */
    static
    void
    my_df(
        const gsl_vector *v,
        void *params,
        gsl_vector *df
    )
    {

        // MS2DSolverImpl<N, M1, M2, M3> *p = (MS2DSolverImpl<N, M1, M2, M3> *) params;

        // Equations eq;
        // for(int i = 0; i < N; ++i){
        //     Symbolic s = p->mRB[i].getP().getS();
        //     for(int j = 0; j < 6; ++j){
        //         eq = (eq, s(j) == gsl_vector_get(v, i*6 + j));
        //     }
        // }

        // for(int i = 0; i < 2*M2; ++i){
        //     eq = (eq, p->qs[i] == gsl_vector_get(v, 6*N + i));
        // }


        // for(int j = 0; j < 6*N; ++j){
        //     double df_value = 0;
        //     for(int i = 0; i < M1 + 3*M2; ++i){
        //         Symbolic c = p->c[i];
        //         Symbolic dc = p->d1c[i][j];
        //         double e = c[eq]*dc[eq];
        //         df_value += e;
        //     }

        //     // rigid body number
        //     int l = j/6;
        //     // position variable component
        //     int k = j%6;
        //     if(k==0 || k==1){
        //         // 
        //     }
        //     else{
        //         double r00 = gsl_vector_get(v, l*6 + 2);
        //         double r01 = gsl_vector_get(v, l*6 + 3);
        //         double r10 = gsl_vector_get(v, l*6 + 4);
        //         double r11 = gsl_vector_get(v, l*6 + 5);

        //         double c1 = (r00*r00 + r01*r01 - 1);
        //         double c2 = (r10*r10 + r11*r11 - 1);
        //         double c3 = (r00*r10 + r01*r11);

        //         switch(k){
        //             case 2:
        //                 // derivative w.r.t. r00
        //                 df_value += c1*2*r00 + c3*r10;
        //                 break;
        //             case 3:
        //                 // derivative w.r.t. r01
        //                 df_value += c1*2*r01 + c3*r11;
        //                 break;
        //             case 4:
        //                 // derivative w.r.t. r10
        //                 df_value += c2*2*r10 + c3*r00;
        //                 break;
        //             case 5:
        //                 // derivative w.r.t. r11
        //                 df_value += c2*2*r11 + c3*r01;
        //                 break;
        //         }
        //     }

        //     gsl_vector_set(df, j, df_value);
    // }

        MS2DSolverImpl<N, M1, M2, M3> *ptr = (MS2DSolverImpl<N, M1, M2, M3> *)params;

        p_t p(gsl_vector_const_ptr(v, 0));
        q_t q(gsl_vector_const_ptr(v, 6*N));
        cextra_t _c = ptr->get_cextra(p, q);

        d1cextra_t _d1c = ptr->get_d1cextra(p, q);
        ddtp_t ddtp = 2*_d1c.transpose()*_c;

        d2cextra_t _d2c = ptr->get_d2cextra(p, q);
        ddtq_t ddtq = 2*_d2c.transpose()*_c;

        for(int i = 0; i < 6*N; ++i)
            gsl_vector_set(df, i, ddtp(i));

        for(int i = 0; i < 2*M2; ++i)
            gsl_vector_set(df, 6*N + i, ddtq(i));

    }

    /* Compute both f and df together. */
    static
    void
    my_fdf(
        const gsl_vector *x
        ,
        void *params
        ,
        double *f
        ,
        gsl_vector *df
    )
    {
        *f = my_f(x, params);
        my_df(x, params, df);
    }

    state_t
    project(
        p_t p = p_t::Zero(),
        q_t q = q_t::Zero(),
        v_t v = v_t::Zero()
    )
    {

        cextra_t _c = get_cextra(p, q);
        std::cout << "Initial error = " << _c.norm() << std::endl;

        size_t iter = 0;
        int status;

        const gsl_multimin_fdfminimizer_type *T;
        gsl_multimin_fdfminimizer *s;

        /* 
            Position of the minimum (1, 2),
            scale factors 10, 20
            height 30
        */

        gsl_vector *x;
        gsl_multimin_function_fdf my_func;

        my_func.n = 6*N + 2*M2;
        my_func.f = MS2DSolverImpl<N, M1, M2, M3>::my_f;
        my_func.df = MS2DSolverImpl<N, M1, M2, M3>::my_df;
        // my_func.df = NULL;
        my_func.fdf = MS2DSolverImpl<N, M1, M2, M3>::my_fdf;
        my_func.params = this;


        /* Starting point, x = (5, 7) */
        x = gsl_vector_alloc(6*N + 2*M2);
        // std::cout << "error " << my_f(x, this) << std::endl;
        // for(int i = 0; i < 6*N + 2*M2; ++i)
        //     gsl_vector_set(x, i, 0.9);

        // gsl_vector_set(x, 0, 0.1);
        // gsl_vector_set(x, 1, 0.2);
        // gsl_vector_set(x, 2, 0.8);
        // gsl_vector_set(x, 3, 0.2);
        // gsl_vector_set(x, 4, 0.1);
        // gsl_vector_set(x, 5, 0.7);

        for(int i = 0; i < 6*N; ++i)
            gsl_vector_set(x, i, p(i));
        for(int i = 0; i < 2*M2; ++i)
            gsl_vector_set(x, 6*N + i, q(i));

        printf("Initial guess\n");
        for(int i = 0; i < 6*N + 2*M2; ++i)
            printf("%.5f ", gsl_vector_get(x, i));
        printf("\n");

        T = gsl_multimin_fdfminimizer_conjugate_fr;
        s = gsl_multimin_fdfminimizer_alloc(T, 6*N + 2*M2);

        /*
         The size of the first trial step is given by step_size = 0.01.
         The accuracy of the line minimization is specified by tol = 1e-6.
         The precise meaning of this parameter depends on the method used.
        */
        gsl_multimin_fdfminimizer_set(s, &my_func, x, 0.01, 1e-3);

        do
        {
            iter++;
            // GSL_ENOPROG
            status = gsl_multimin_fdfminimizer_iterate(s);

            if (status)
            break;

            status = gsl_multimin_test_gradient(s->gradient, 1e-5);

            if(status == GSL_SUCCESS)
                printf("Minimum found at:\n");

            printf(
                "%5d %10.5f\n",
                static_cast<int>(iter),
                s->f
            );

            for(int i = 0; i < 6*N + 2*M2; ++i)
                printf("%.5f ", gsl_vector_get(s->x, i));
            printf("\n");

        }
        while (status == GSL_CONTINUE && iter < 100);



        // p_t p_out = p_t(s->x->data);
        Eigen::Matrix<double, 6*N + 2*M2, 1> p_and_q(s->x->data);

        p_t p0 = p_and_q.segment(0,6*N);
        q_t q0 = p_and_q.segment(6*N,2*M2);
        projectSolver(
            p0, q0
        );
        p_and_q << p0, q0;

        // v_t v = v_t::Zero();
        // v(0) = 10;
        // v(1) = -10;
        // Eigen::Matrix<double, m, 3*N> dkc = getdkc(p_out);
        // Eigen::Matrix<double, m, m> to_invert = dkc*dkc.transpose();
        // v_t v_out = v - dkc.transpose()*to_invert.llt().solve(dkc*v);
        // v_t v_out = v;
        v_t v_out = getProjectedVelocity(
            p_and_q.segment(0, 6*N)
            ,
            p_and_q.segment(6*N, 2*M2)
            ,
            v
        );

        state_t out;
        out.segment(0, 6*N + 2*M2) = p_and_q;
        out.segment(6*N + 2*M2, 3*N) = v_out;

        // out(0) += 10;

        // state_t out;
        // out.segment(0, 6*N) = p;
        // out.segment(6*N, 2*M2) = q;
        // out.segment(6*N + 2*M2, 3*N) = v;

        gsl_multimin_fdfminimizer_free(s);
        gsl_vector_free(x);

        return out;
    }

    static
    int
    func(
        double t,
        const double y[],
        double f[],
        void *params
    )
    {
        (void)(t); /* avoid unused parameter warning */
        MS2DSolverImpl<N, M1, M2, M3> *p = (MS2DSolverImpl<N, M1, M2, M3> *) params;

        /* this does not work:
        p->getD(state_t(y)) is a temporary object
        and the operation 'f = p->getD(state_t(y)).data()' does not copy anything
        so f will be pointing to just zeros,
        which is the default initialization
        */
        // f = p->getD(state_t(y)).data();

        state_t x = p->getD(state_t(y));
        for(int i = 0; i < (6+3)*N + 2*M2; ++i)
            f[i] = x(i);

        return GSL_SUCCESS;
    }

    int
    solve(
        int K = 10
    )
    {
        //   gsl_odeiv2_system sys = {func, jac, (6+3)*N, &this};
        gsl_odeiv2_system sys = {MS2DSolverImpl<N, M1, M2, M3>::func, NULL, (6+3)*N + 2*M2, this};

        gsl_odeiv2_driver * d = gsl_odeiv2_driver_alloc_y_new(
            &sys,
            gsl_odeiv2_step_rk8pd,
            1e-6,
            1e-6,
            0.0
        );

        double t = 0.0, t1 = 1.0;

        // problem with this
        // double *y = project().data();

        double yall[K+1][(6+3)*N + 2*M2];

        double y[(6+3)*N + 2*M2];
        state_t x0 = project();
        for(int i = 0; i < (6+3)*N + 2*M2; ++i)
            y[i] = x0(i);

        for(int j = 0; j < (6+3)*N + 2*M2; ++j)
            yall[0][j] = y[j];

        for (int i = 1; i <= K; i++)
        {

            printf ("%.5e %.5e %.5e\n", t, y[0], y[1]);

            double ti = i * t1 / static_cast<double>(K);
            int status = gsl_odeiv2_driver_apply(d, &t, ti, y);

            for(int j = 0; j < (6+3)*N; ++j)
                yall[i][j] = y[j];

            if (status != GSL_SUCCESS)
            {
                printf ("error, return value=%d\n", status);
                break;
            }

        }

        gsl_odeiv2_driver_free(d);
        return 0;
    }

    virtual
    int solverImplementation(
        double time_initial,
        double time_step,
        int steps,
        double** yall,
        int * current_step
    ){
        //   gsl_odeiv2_system sys = {func, jac, (6+3)*N, &this};
        gsl_odeiv2_system sys = {MS2DSolverImpl<N, M1, M2, M3>::func, NULL, (6+3)*N + 2*M2, this};

        gsl_odeiv2_driver * d = gsl_odeiv2_driver_alloc_y_new(
            &sys,
            gsl_odeiv2_step_rk8pd,
            1e-6,
            1e-6,
            0.0
        );

        double t = time_initial;
        double t1 = time_initial + steps*time_step;

        double y[(6+3)*N + 2*M2];
        state_t x0 = project(p0, q0, v0);

        for(int i = 0; i < (6+3)*N + 2*M2; ++i)
            y[i] = x0(i);

        for(int j = 0; j < (6+3)*N + 2*M2; ++j)
            yall[0][j] = y[j];

        for (int i = 0; i < steps; i++)
        {
            *current_step = i;
            c_t _c = get_c(p_t(&y[0]), q_t(&y[6*N]));

            // printf ("%.5e %.5e %.5e\n", t, y[0], y[1]);

            double ti = time_initial + (i + 1) * time_step;
            int status = gsl_odeiv2_driver_apply(d, &t, ti, y);
            // ddtv_t forces = getInternalForces(state_t(y));
            ddtv_t forces = ddtv_t::Zero();

            for(int j = 0; j < (6+3)*N + 2*M2; ++j)
                yall[i][j] = y[j];
            for(int j = 0; j < 3*N; ++j)
                yall[i][(6+3)*N  + 2*M2 + j] = forces(j);

            if (status != GSL_SUCCESS)
            {
                printf ("error, return value=%d\n", status);
                break;
            }

        }

        gsl_odeiv2_driver_free(d);
        return 0;
    };

    int projectSolver(
        p_t & p0,
        q_t & q0,
        int steps = 300,
        double time_step = 0.01
    ){
        //   gsl_odeiv2_system sys = {func, jac, (6+3)*N, &this};
        gsl_odeiv2_system sys = {MS2DSolverImpl<N, M1, M2, M3>::getPositionSolver, NULL, 6*N + 2*M2, this};

        gsl_odeiv2_driver * d = gsl_odeiv2_driver_alloc_y_new(
            &sys,
            gsl_odeiv2_step_rk8pd,
            1e-6,
            1e-6,
            0.0
        );

        c_t _c1 = get_c(p0, q0);
        cextra_t _cextra1 = get_cextra(p0, q0);

        double t = 0;

        double y[6*N + 2*M2];

        for(int i = 0; i < 6*N; ++i)
            y[i] = p0(i);

        for(int i = 0; i < 2*M2; ++i)
            y[6*N + i] = q0(i);

        for (int i = 0; i < steps; i++)
        {


            double ti = (i + 1) * time_step;
            int status = gsl_odeiv2_driver_apply(d, &t, ti, y);

            if (status != GSL_SUCCESS)
            {
                printf ("error, return value=%d\n", status);
                break;
            }

        }

        for(int i = 0; i < 6*N; ++i)
            p0(i) = y[i];

        for(int i = 0; i < 2*M2; ++i)
            q0(i) = y[6*N + i];

        c_t _c2 = get_c(p0, q0);
        cextra_t _cextra2 = get_cextra(p0, q0);

        gsl_odeiv2_driver_free(d);
        return 0;
    };

    static
    int
    getPositionSolver(
        double t,
        const double y[],
        double ddty[],
        void *params
    ){

        (void)(t); /* avoid unused parameter warning */
        MS2DSolverImpl<N, M1, M2, M3> *ptr = (MS2DSolverImpl<N, M1, M2, M3> *) params;

        Eigen::Matrix<double, 6*N + 2*M2, 1> pandq(y);
        p_t p = pandq.segment(0, 6*N);
        q_t q = pandq.segment(6*N, 2*M2);

        // add stability terms
        // for stability terms, we must consider the non holonomic constraints
        c_t _c = ptr->get_c(p, q);

        // d1c_t A = ptr->get_d1c(p, q);
        // d2c_t B = ptr->get_d2c(p, q);

        // // _c should be zero, in which case, stability = 0
        // Eigen::Matrix<double, M1+3*M2+M3, 6*N + 2*M2> K;
        // K << A, B;
        // Eigen::Matrix<double, 6*N + 2*M2, 1> stability = -K.transpose()*(K*K.transpose()).llt().solve(_c);

        // ddtp_t p_dot = stability.segment(0, 6*N);
        // ddtq_t q_dot = stability.segment(6*N, 2*M2);


        d1ck_t A = ptr->get_d1ck(p, q);
        d2c_t B = ptr->get_d2c(p, q);

        Eigen::Matrix<double, M1+3*M2+M3, 3*N + 2*M2> K;
        K << A, B;
        Eigen::Matrix<double, 3*N + 2*M2, 1> stability = -K.transpose()*(K*K.transpose()).llt().solve(_c);

        v_t v_stability = stability.segment(0, 3*N);
        ddtp_t p_dot = ptr->ddtP(p, v_stability);
        ddtq_t q_dot = stability.segment(3*N, 2*M2);

        Eigen::Matrix<double, 6*N + 2*M2, 1> pandq_dot;
        pandq_dot << p_dot, q_dot;

        // does not work
        // ddty = pandq_dot.data();

        for(int i = 0; i < 6*N + 2*M2; ++i)
            ddty[i] = pandq_dot(i);

        return GSL_SUCCESS;
    }

    p_t p0;
    q_t q0;
    v_t v0;

private:
    /**
     * @brief position
     * 
     */
    std::string mName;

    /* inertia matrix */
    Eigen::Matrix<double, 3*N, 3*N> mI;
    Eigen::Matrix<double, 3*N, 3*N> mIinv;
    std::list<int> mListWithContact;


};

template<int N, int M1, int M2, int M3>
int
func(
    double t,
    const double y[],
    double ddty[],
    void * params
)
{

    MS2DSolverImpl<N, M1, M2, M3> r = *(MS2DSolverImpl<N, M1, M2, M3> *)params;
    return r.functionGSL(t, y, ddty, params);

}




