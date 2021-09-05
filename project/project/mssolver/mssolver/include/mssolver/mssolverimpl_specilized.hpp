#pragma once

// std::array
#include  <array>

// std::string
#include <string>

// eigen for matrix vector operations
#include <Eigen/Dense>

// gsl for solving differential equations
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

// gsl for minimization
#include <gsl/gsl_multimin.h>

// to get utilitarian functions
#include "mssolver/util.hpp"

// IMS2DSolver
#include <msinterface/solver.hpp>

#include <msinterface/exceptions.hpp>

/**
 * @brief Mechanical system is a (3*N - M1)-th dimensional system
 * 
 * @tparam N number of rigid bodies
 * @tparam M1 number of holonomic constraints
 */
template<int N, int M1>
class MS2DHOLSolverImpl:
public IMS2DSolver
{
public:

    /**
     * @brief Construct a solver for a mechanical system with N rigid bodies
     * 
     * @param ptr Pointer to object that holds contraints
     * @param inertias Inertias (linear and angular) of rigid bodies
     * @param name Name of mechanical system
     * @param rigid_body_names Name of each rigid body
     */
    MS2DHOLSolverImpl(
        std::array<rbi_t, N> inertias
        ,
        std::string name
        ,
        std::array<std::string, N> rigid_body_names
    ):
    IMS2DSolver()
    ,
    mName(name)
    ,
    mRBNames(rigid_body_names)
    ,
    mI(Eigen::Matrix<double, 3*N, 3*N>::Zero())
    ,
    mIinv(Eigen::Matrix<double, 3*N, 3*N>::Zero())
    ,
    mListWithContact(std::list<int>(0))
    ,
    mVelocityPlotFactor(std::array<double,2>({{1,1}}))
    ,
    mForcePlotFactor(std::array<double,2>({{1,1}}))
    ,
    mGravity(std::array<double,2>({{0, -9.8*1000}})) // force in kg mm / s / s (using millimeters instead of meters)
    {

        for(int i = 0; i < N; ++i){
            setLinearInertia(inertias[i][0], i);
            setAngularInertia(inertias[i][1], i);
        }

    }

    /**
     * @brief Get the name of the mechical system
     * 
     * @return std::string name of the mechanical system
     */
    virtual
    std::string getName() const {
        return mName;
    }

    /**
     * @brief Get the name of the ith rigid body
     * 
     * @param i rigid body id (between 0 and N)
     * @return std::string name of the ith rigid body
     */
    virtual
    std::string getName(
        int i // rigid body number
    ) const {
        validateId(i);
        return mRBNames[i];
    }

    /**
     * @brief Get the mass of the ith rigid body (linear inertia)
     * 
     * @param i rigid body id
     * @return double value of the mass
     */
    virtual
    double getLinearInertia(
        int i // rigid body number
    ) const {
        validateId(i);
        return mI(3*i, 3*i);
    }

    /**
     * @brief Set the mass (the linear inertia) of the ith rigid body
     * 
     * @param mass mass value
     * @param i rigid body id
     */
    virtual
    void setLinearInertia(
        double mass,
        int i // rigid body number
    ){
        validateId(i);

        mI(3*i + 0, 3*i + 0) = mass;
        mI(3*i + 1, 3*i + 1) = mass;

        mIinv(3*i + 0, 3*i + 0) = 1./mass;
        mIinv(3*i + 1, 3*i + 1) = 1./mass;

        // Eigen::Matrix<double, 2, 2> m;
        // m << 
        //     mass, 0,
        //     0, mass;
        // mI.block(3*i, 3*i, 2, 2) = m;

        // Eigen::Matrix<double, 2, 2> minv;
        // minv << 
        //     1./mass, 0,
        //     0, 1./mass;
        // mI.block(3*i, 3*i, 2, 2) = minv;
    }

    /**
     * @brief Get the moment of inertia of the ith rigid body (angular inertia)
     * 
     * @param i rigid body id
     * @return double value of the moment of inertia
     */
    virtual
    double getAngularInertia(
        int i // rigid body number
    ) const {
        validateId(i);
        return mI(3*i + 2, 3*i + 2);
    }

    /**
     * @brief Set the moment of inertia (the angular inertia) of the ith rigid body
     * 
     * @param moment_of_inertia moment of inertia value
     * @param i rigid body id
     */
    virtual
    void setAngularInertia(
        double moment_of_inertia,
        int i // rigid body number
    ) {
        mI(3*i + 2, 3*i + 2) = moment_of_inertia;
        mIinv(3*i + 2, 3*i + 2) = 1./moment_of_inertia;
    }

    /**
     * @brief Get the gravity force acting on each rigid body
     * 
     * @return std::array<double, 2> Gravity force
     */
    virtual
    std::array<double, 2> getGravity(
    ) const {
        return mGravity;
    };

    /**
     * @brief Set the gravity force acting on each rigid body
     * 
     * @param gravity Gravity force
     */
    virtual
    void setGravity(
        std::array<double, 2> gravity
    ){
        mGravity = gravity;
    };

    /**
     * @brief Compute time derivative of pose of a rigid body, given its twist
     * 
     * @details Pose is composed of a linear and angular components, and twist is also composed of a linear and angular components
     * 
     * @param p pose of a rigid body
     * @param v twist of a rigid body
     * @return Eigen::Matrix<double, 2+4, 1> 
     */
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

        // terms on the right side are stability terms
        // they guarantee a rotation matrix remains a rotation matrix
        ddtp(2) = +v(2)*p(3) - p(2)*e1           - p(3)*e3;
        ddtp(3) = -v(2)*p(2)           - p(3)*e2 - p(2)*e3;
        ddtp(4) = +v(2)*p(5) - p(4)*e1           - p(5)*e3;
        ddtp(5) = -v(2)*p(4)           - p(5)*e2 - p(4)*e3;
        return ddtp;
    }

    /**
     * @brief Size of solution when solving differential equation in solverImplementation
     * 
     * @return int Size of solution when solving differential equation in solverImplementation
     */
    virtual int dimSolution() const {
        return 
            6*N  +  // positions
            3*N  +  // velocities
            3*N     // forces
            ;
    }

    /**
     * @brief Number of rigid bodies
     * 
     * @return int Number of rigid bodies
     */
    virtual int dim() const {return N;}

    /**
     * @brief Number of rigid bodies
     * 
     * @return int Number of rigid bodies
     */
    virtual int dimRigidBodies() const {return N;}

    /**
     * @brief Number of constraints
     * 
     * @return int Number of constraints
     */
    virtual int dimConstraints() const {return M1;}

    /**
     * @brief Number of contact contraints
     * 
     * @return int Number of contact contraints
     */
    virtual int dimContactConstraints() const {return M1;}

    /**
     * @brief Number of non sliding contact contraints
     * 
     * @return int Number of non sliding contact contraints
     */
    virtual int dimNonSlidingContactConstraints() const {return 0;}

    /**
     * @brief Get the position (for plotting purposes) of a rigid body from the mechanical system
     * 
     * @param solution Pointer to solution coming from solverImplementation
     * @param k Rigid body id
     * @return rbp_t Rigid body position
     */
    virtual
    rbp_t getPositionPlot(
        double* solution,
        int k // rigid body number
    ) const {
        return rbp_t(&solution[6*k + 0]);
    }

    /**
     * @brief Set the linear and angular velocity factors, used for plotting purposes
     * 
     * @param factors Linear and angular factors
     */
    virtual
    void
    setVelocityPlotFactor(
        std::array<double, 2> factors
    ){
        mVelocityPlotFactor = factors;
    }

    /**
     * @brief Get the linear and angular velocity factors
     * 
     */
    virtual
    std::array<double, 2>
    getVelocityPlotFactor(
        // 
    ) const {
        return mVelocityPlotFactor;
    }

    /**
     * @brief Get the velocity (for plotting purposes) of a rigid body from the mechanical system
     * 
     * @param solution Pointer to solution coming from solverImplementation
     * @param k Rigid body id
     * @return rbv_t Rigid body velocity
     */
    virtual
    rbv_t getVelocityPlot(
        double* solution,
        int k // rigid body number
    ) const {
        // return rbv_t(&solution[6*N + 3*k]);
        rbv_t out;
        out << 
        solution[6*N + 3*k + 0]*mVelocityPlotFactor[0], // linear
        solution[6*N + 3*k + 1]*mVelocityPlotFactor[0], // linear
        solution[6*N + 3*k + 2]*mVelocityPlotFactor[1]; // angular
        return out;
    }

    /**
     * @brief Set the linear and angular force factors, used for plotting purposes
     * 
     * @param factors Linear and angular factors
     */
    virtual
    void
    setForcePlotFactor(
        std::array<double, 2> factors
    ){
        mForcePlotFactor = factors;
    }

    /**
     * @brief Get the linear and angular force factors
     * 
     */
    virtual
    std::array<double, 2>
    getForcePlotFactor(
        // 
    ) const {
        return mForcePlotFactor;
    }

    /**
     * @brief Get the force (for plotting purposes) of a rigid body from the mechanical system
     * 
     * @param solution Pointer to solution coming from solverImplementation
     * @param k Rigid body id
     * @return rbf_t Rigid body force
     */
    virtual
    rbf_t getForcePlot(
        double* solution,
        int k // rigid body number
    ) const {
        // return rbf_t(&solution[6*N + 3*N + 3*k]);
        rbv_t out;
        out << 
        solution[6*N + 3*N + 3*k + 0]*mForcePlotFactor[0], // linear
        solution[6*N + 3*N + 3*k + 1]*mForcePlotFactor[0], // linear
        solution[6*N + 3*N + 3*k + 2]*mForcePlotFactor[1]; // angular
        return out;
    }

    /**
     * @brief Get the inertial vectors (for plotting purposes) of a rigid body from the mechanical system
     * 
     * @details This method should be overwritten by children if necessary
     * 
     * @param solution Pointer to solution coming from solverImplementation
     * @param k Rigid body id
     * @return std::vector<IMS2DSolver::v_t> Vector of inertial forces
     */
    virtual
    std::vector<IMS2DSolver::v_t> getInertialVectorsPlot(
        double* solution,
        int k // rigid body number
    ) const {
        return std::vector<IMS2DSolver::v_t>({});
    }

    /**
     * @brief Get the body vectors (for plotting purposes) of a rigid body from the mechanical system
     * 
     * @details This method should be overwritten by children if necessary
     * 
     * @param solution Pointer to solution coming from solverImplementation
     * @param k Rigid body id
     * @return std::vector<IMS2DSolver::v_t> Vector of body forces
     */
    virtual
    std::vector<IMS2DSolver::v_t> getBodyVectorsPlot(
        double* solution,
        int k // rigid body number
    ) const {

        return std::vector<IMS2DSolver::v_t>({});
    }

    /**
     * @brief Get the position (for diplaying numbers purposes) of rigid body of the mechanical system
     * 
     * @param solution Pointer to solution coming from solverImplementation
     * @param k Rigid body id
     * @return std::array<double, 3> Linear position + angular position in degrees
     */
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

    /**
     * @brief Get the velocity (for diplaying numbers purposes) of rigid body of the mechanical system
     * 
     * @param solution Pointer to solution coming from solverImplementation
     * @param k Rigid body id
     * @return std::array<double, 3> Linear velocity + angular velocity in degrees per second
     */
    virtual
    std::array<double, 3> getVelocity(
        double* solution,
        int k // rigid body number
    ) const {
        return std::array<double, 3>(
            {{
                solution[6*N + 3*k + 0],
                solution[6*N + 3*k + 1],
                180/M_PI*solution[6*N + 3*k + 2] // radians to degrees
            }}
        );
    }

    /**
     * @brief Get the force (for diplaying numbers purposes) of rigid body of the mechanical system
     * 
     * @param solution Pointer to solution coming from solverImplementation
     * @param k Rigid body id
     * @return std::array<double, 3> Linear force + angular force in degrees (instead of radians)
     */
    virtual
    std::array<double, 3> getForce(
        double* solution,
        int k // rigid body number
    ) const {
        return std::array<double, 3>(
            {{
                solution[6*N + 3*N + 3*k + 0],
                solution[6*N + 3*N + 3*k + 1],
                180/M_PI*solution[6*N + 3*N + 3*k + 2] // radians to degrees
            }}
        );
    }

    /**
     * @brief Get the position (for diplaying numbers purposes) of a specific point in a rigid body of the mechanical system
     * 
     * @param solution Pointer to solution coming from solverImplementation
     * @param k Rigid body id
     * @return std::array<double, 2> Position of that point
     */
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

    /**
     * @brief Get the velocity (for diplaying numbers purposes) of a specific point in a rigid body of the mechanical system
     * 
     * @param solution Pointer to solution coming from solverImplementation
     * @param k Rigid body id
     * @return std::array<double, 2> Velocity of that point
     */
    virtual
    std::array<double, 2> getVelocityOfPoint(
        double* solution,
        int k, // rigid body number
        std::array<double, 2> relative_position
    ) const {
        double v[2] = {solution[6*N + 3*k + 0], solution[6*N + 3*k + 1]};
        double omega = solution[6*N + 3*k + 2];
        double r[2][2] = {
                {solution[6*k + 2], solution[6*k + 3]},
                {solution[6*k + 4], solution[6*k + 5]}
            };
        return std::array<double, 2>(
            {{
                v[0] + omega*(+r[0][1]*relative_position[0] - r[0][0]*relative_position[1]),
                v[1] + omega*(+r[1][1]*relative_position[0] - r[1][0]*relative_position[1])
            }}
        );
    }

    /**
     * @brief Position of the mechanical system
     * 
     */
    typedef Eigen::Matrix<double, 6*N, 1> p_t;
    /**
     * @brief Time derivative of the position of the mechanical system
     * 
     */
    typedef Eigen::Matrix<double, 6*N, 1> ddtp_t;
    /**
     * @brief Points of contact of the different rigid bodies of the mechanical system
     * 
     */
    typedef Eigen::Matrix<double, 3*N, 1> v_t;
    /**
     * @brief Velocity of the mechanical system
     * 
     */
    typedef Eigen::Matrix<double, 3*N, 1> velocity_t;
    /**
     * @brief Time derivative of the velocity of the mechanical system
     * 
     */
    typedef Eigen::Matrix<double, 3*N, 1> ddtv_t;
    /**
     * @brief State of the mechanical system
     * 
     */
    typedef Eigen::Matrix<double, 6*N + 3*N, 1> state_t;
    /**
     * @brief Time derivative of the state of the mechanical system
     * 
     */
    typedef Eigen::Matrix<double, 6*N + 3*N, 1> ddtstate_t;
    /**
     * @brief Force applied on the mechanical system
     * 
     */
    typedef Eigen::Matrix<double, 3*N, 1> f_t;
    /**
     * @brief Number of degress of freedom when choosing the position and the contact points
     * 
     */
    Eigen::Matrix<double, 3*N - M1, 1> pdof_t;

    /**
     * @brief Constraint that defines the mechanical system
     * 
     * @details c(p)
     * 
     */
    typedef Eigen::Matrix<double, M1, 1> c_t;
    /**
     * @brief Derivative of the constraints with respect to the position
     * 
     * @details d1c(p) = d/dp c(p)
     * 
     */
    typedef Eigen::Matrix<double, M1, 6*N> d1c_t;
    /**
     * @brief Derivative of the constraints with respect to the position after the position derivative
     * 
     * @details d1c(p) k(p) where d/dt p = k(p) v
     * 
     */
    typedef Eigen::Matrix<double, M1, 3*N> d1ck_t;
    /**
     * @brief Time derivative of d1c(p) k(p)
     * 
     * @details d/dt (d1c(p) k(p)) =  d/dp (d1c(p,q) k(p)) d/dtp
     * 
     */
    typedef Eigen::Matrix<double, M1, 1> ddtd1ck_t;

    /**
     * @brief Constraint that defines the mechanical system + constraints on rotation matrices
     * 
     * @details c(p) + constrainst on R1 + ... + constrainst on R2, where Ri is the rotation matrix of rigid body i
     * 
     */
    typedef Eigen::Matrix<double, M1 + 3*N, 1> cextra_t;
    /**
     * @brief Derivative of the constraints+extra with respect to the position
     * 
     */
    typedef Eigen::Matrix<double, M1 + 3*N, 6*N> d1cextra_t;

    /**
     * @brief Get the position from the state
     * 
     * @param x State
     * @return p_t Position
     */
    p_t getP(state_t const & x) {
        return x.segment(0, 6*N);
    }

    /**
     * @brief Get the velocity from the state
     * 
     * @param x State
     * @return v_t Velocity
     */
    v_t getV(state_t const & x) {
        return x.segment(6*N, 3*N);
    }

    /**
     * @brief Get the position time derivative
     * 
     * @param p Mechnical system position
     * @param v Mechnical system velocity
     * @return ddtp_t Time derivative of the position
     */
    ddtp_t
    ddtP(
        p_t const & p
        ,
        v_t const & v
    ){

        ddtp_t ddtp = ddtp_t::Zero();

        for(int i = 0; i < N; ++i){

            Eigen::Matrix<double, 6, 1> pi = p.segment(6*i, 6);
            Eigen::Matrix<double, 3, 1> vi = v.segment(3*i, 3);

            ddtp.segment(6*i, 6) = VelocityddtP(pi, vi);
        }
        return ddtp;
    }

    /**
     * @brief Get the position time derivative
     * 
     * @param x Mechanical system state
     * @return ddtp_t Time derivative of the position
     */
    ddtp_t
    ddtP(
        state_t const & x
    ){
        return ddtP(getP(x), getV(x));
    }

    /**
     * @brief Get the internal forces applied to the mechanical system
     * 
     * @param time Time instant
     * @param state State of the mechanical system
     * @return f_t Force
     */
    f_t
    getInternalForces(
        double time,
        state_t const & state
    ){

        p_t p = state.segment(0, 6*N);
        v_t v = state.segment(6*N, 3*N);


        d1ck_t A = get_d1ck(p);
        ddtp_t p_dot = ddtP(p, v);

        // ddt(A) v = (dP(A) Pdot) v
        Eigen::Matrix<double, M1, 1> dA = get_ddtd1ck(p, p_dot, v);

        Eigen::Matrix<double, M1, M1> to_invert;
        d1ck_t NS = A;
        to_invert = NS*mIinv*NS.transpose();

        f_t force_input = getForce(time, state);

        // acceleration no constraints
        ddtv_t free_acceleration = mIinv*force_input;

        f_t f;
        f =
        -mIinv*NS.transpose()*to_invert.llt().solve(
            NS*free_acceleration
            +
            dA
        );

        return f;
    }

    /**
     * @brief Get the force applied on the ith rigid body
     * 
     * @param k Rigid body id
     * @param time Time instant
     * @return IMS2DSolver::rbf_t Force applied on rigid body
     */
    virtual
    IMS2DSolver::rbf_t
    getForceRB(
        int k,
        double time
    ){
        IMS2DSolver::rbf_t f = IMS2DSolver::rbf_t::Zero();
        f << mGravity[0], mGravity[1], 0; // kg mm / s / s
        return f;
    }

    /**
     * @brief Get the force applied on the mechanical system
     * 
     * @param time Time instant
     * @param state State of the mechanical system
     * @return f_t Force applied on the mechanical system
     */
    virtual
    f_t getForce(
        double time,
        state_t const & state
    ){
        f_t f;
        for(int i = 0; i < N; ++i)
            f.segment(3*i, 3) = getForceRB(i, time);
        return f;
    }

    /**
     * @brief Get a velocity that lives in the correct nullspace
     * 
     * @param p Position of the mechanical system
     * @param v Velocity to be projected
     * @return v_t Velocity that lives in the correct nullspace
     */
    v_t
    getProjectedVelocity(
        p_t const & p
        ,
        v_t const & v
    ){
        d1ck_t NS = get_d1ck(p);
        Eigen::Matrix<double, M1, M1> to_invert = NS*NS.transpose();
        v_t v_projected = v - NS.transpose()*to_invert.llt().solve(NS*v);
        return v_projected;
    }

    /**
     * @brief Get the state time derivative
     * 
     * @param time Time instant
     * @param state State of the mechanical system
     * @return ddtstate_t Time derivative of the state
     */
    ddtstate_t getStateDot(
        double time,
        state_t const & state
    ){

        p_t p = state.segment(0, 6*N);
        v_t v = state.segment(6*N, 3*N);

        d1ck_t A = get_d1ck(p);

        // add stability terms
        c_t _c = get_c(p);
        // _c should be zero, in which case, stability = 0
        v_t v_stability = -A.transpose()*(A*A.transpose()).llt().solve(_c);

        // without stability
        // ddtp_t ddtp = ddtP(p, v);
        // with stability
        ddtp_t ddtp = ddtP(p, v + v_stability);

        // ddt(A) v = (dP(A) Pdot) v
        Eigen::Matrix<double, M1, 1> dA = get_ddtd1ck(p, ddtp, v);

        Eigen::Matrix<double, M1, M1> to_invert;
        d1ck_t NS = A;
        to_invert = NS*mIinv*NS.transpose();

        f_t f = getForce(time, state);
        // acceleration no constraints
        ddtv_t free = mIinv*f;

        ddtv_t ddtv;
        ddtv =
        free -
        mIinv*NS.transpose()*to_invert.llt().solve(
            NS*(free + v)
            +
            dA
        );

        ddtstate_t ddtstate;
        ddtstate.segment(0  , 6*N ) = ddtp;
        ddtstate.segment(6*N, 3*N ) = ddtv;
        return ddtstate;

    }

    /**
     * @brief Get how close to zero constraints are satisfied
     * 
     * @param state State of the mechanical system
     * @return double norm of the constraints
     */
    double
    gePositionDomainError(
        state_t const & state
    ){
        p_t p = state.segment(0, 6*N);
        
        cextra_t cextra = get_cextra(p);
        return cextra.norm();
    }

    /**
     * @brief Get how close to the velocity is to the nullspace
     * 
     * @param state State of the mechanical system
     * @return double Norm of vector that quantifies how close to the velocity is to the nullspace
     */
    double
    getVelocityDomainError(
        state_t const & state
    ){
        p_t p = state.segment(0, 6*N);
        v_t v = state.segment(6*N, 3*N);

        d1ck_t A = get_d1ck(p);
        Eigen::Matrix<double, M1, 1> error = A*v;
        return error.norm();
    }

    /**
     * @brief Get the constraints satisfaction: c(p)
     * 
     * @param p Position
     * @return c_t Constraints
     */
    virtual c_t get_c(p_t const & p) const = 0;
    /**
     * @brief Get the derivative of the constraints with respect to the position: d/dp c(p)
     * 
     * @param p Position
     * @return d1c_t Derivative
     */
    virtual d1c_t get_d1c(p_t const & p) const = 0;
    /**
     * @brief Get the derivative of the constraints with respect to the position after the position derivative: d/dp c(p) k(p)
     * 
     * @param p Position
     * @return d1ck_t Derivative
     */
    virtual d1ck_t get_d1ck(p_t const & p) const = 0;
    /**
     * @brief Get the time derivative of d1ck(p)
     * 
     * @param p Position
     * @param ddtp Time derivative of position
     * @param v Velocity
     * @return ddtd1ck_t Derivative
     */
    virtual ddtd1ck_t get_ddtd1ck(p_t const & p, ddtp_t const & ddtp, v_t const & v) const = 0;

    /**
     * @brief Get the constraints+extra satisfaction: c_extra(p)
     * 
     * @param p Position
     * @return c_t Constraints+extra
     */
    virtual cextra_t get_cextra(p_t const & p) const = 0;
    /**
     * @brief Get the derivative of the constraints+extra with respect to the position: d/dp c_extra(p)
     * 
     * @param p Position
     * @return d1cextra_t Derivative
     */
    virtual d1cextra_t get_d1cextra(p_t const & p) const = 0;

    /*************************************************************************/
    /* GSL minimization solver                                               */
    /*************************************************************************/

    /**
     * @brief Function which we which to minimize
     * 
     * @param v Vector whose value we wish to find where the function is minimized
     * @param params It will be a pointer to an object of this class
     * @return double Value of the function at the currect vector
     */
    static
    double
    gsl_errorfunction(
        const gsl_vector *v
        ,
        void *params
    )
    {
        MS2DHOLSolverImpl<N, M1> *p = (MS2DHOLSolverImpl<N, M1> *)params;

        cextra_t _c = p->get_cextra(
            p_t(gsl_vector_const_ptr(v,  0))
        );

        return pow(_c.norm(), 2);

    }

    /**
     * @brief Derivative of the function which we which to minimize
     * 
     * @details The gradient of f(x1, ..., xn), df = (df/dx1, ... , df/dxn).
     * 
     * @param v Vector whose value we wish to find where the function is minimized
     * @param params It will be a pointer to an object of this class
     * @param df It will store the derivative
     */
    static
    void
    gsl_derrorfunction(
        const gsl_vector *v,
        void *params,
        gsl_vector *df
    )
    {


        MS2DHOLSolverImpl<N, M1> *ptr = (MS2DHOLSolverImpl<N, M1> *)params;

        p_t p(gsl_vector_const_ptr(v, 0));
        cextra_t _c = ptr->get_cextra(p);

        d1cextra_t _d1c = ptr->get_d1cextra(p);
        ddtp_t ddtp = 2*_d1c.transpose()*_c;

        for(int i = 0; i < 6*N; ++i)
            gsl_vector_set(df, i, ddtp(i));

    }

    /**
     * @brief Function and its derivative, where we which to minimize the function
     * 
     * @details The gradient of f(x1, ..., xn), df = (df/dx1, ... , df/dxn).
     * 
     * @param v Vector whose value we wish to find where the function is minimized
     * @param params It will be a pointer to an object of this class
     * @param f Value of the function
     * @param df It will store the derivative
     */
    static
    void
    gsl_errorfunction_complete(
        const gsl_vector *v
        ,
        void *params
        ,
        double *f
        ,
        gsl_vector *df
    )
    {
        *f = gsl_errorfunction(v, params);
        gsl_derrorfunction(v, params, df);
    }

    /**
     * @brief Find position and velocity that satisfies constraints by using a minimizing function from gsl
     * 
     * @param p Initial guess for position
     * @param v Initial guess for velocity
     * @return state_t Final state that satisfies constraints
     */
    state_t
    projectMinimizer(
        p_t const & p = p_t::Zero(),
        v_t const & v = v_t::Zero()
    )
    {

        size_t iter = 0;
        int status;

        const gsl_multimin_fdfminimizer_type *T;
        gsl_multimin_fdfminimizer *s;

        // create and populate vector
        gsl_vector *x;
        x = gsl_vector_alloc(6*N);
        for(int i = 0; i < 6*N; ++i)
            gsl_vector_set(x, i, p(i));

        gsl_multimin_function_fdf func2min;
        func2min.n = 6*N;
        func2min.f = MS2DHOLSolverImpl<N, M1>::gsl_errorfunction;
        func2min.df = MS2DHOLSolverImpl<N, M1>::gsl_derrorfunction;
        // func2min.df = NULL;
        func2min.fdf = MS2DHOLSolverImpl<N, M1>::gsl_errorfunction_complete;
        func2min.params = this;

        printf("Initial:\n");
        projectMinimizerPrintInfo(p);

        T = gsl_multimin_fdfminimizer_conjugate_fr;
        s = gsl_multimin_fdfminimizer_alloc(T, 6*N);

        /*
         The size of the first trial step is given by step_size = 0.01.
         The accuracy of the line minimization is specified by tol = 1e-6.
         The precise meaning of this parameter depends on the method used.
        */
        gsl_multimin_fdfminimizer_set(s, &func2min, x, 0.01, 1e-3);

        do
        {
            iter++;
            // GSL_ENOPROG
            status = gsl_multimin_fdfminimizer_iterate(s);

            if (status)
            break;

            status = gsl_multimin_test_gradient(s->gradient, 1e-5);

            // if(status == GSL_SUCCESS)
            //     printf("Minimum found at:\n");

            // printf(
            //     "iteration = %5d, value = %10.5f\n",
            //     static_cast<int>(iter),
            //     s->f
            // );

            // for(int i = 0; i < 6*N; ++i)
            //     printf("%.5f ", gsl_vector_get(s->x, i));
            // printf("\n");

        }
        while (status == GSL_CONTINUE && iter < 100);

        p_t pfinal(s->x->data);

        printf("Final:\n");
        projectMinimizerPrintInfo(pfinal);

        v_t vfinal = getProjectedVelocity(
            pfinal,
            v
        );

        state_t out;
        out.segment(0, 6*N) = pfinal;
        out.segment(6*N, 3*N) = vfinal;

        gsl_multimin_fdfminimizer_free(s);
        gsl_vector_free(x);

        return out;
    }

    /**
     * @brief Useful printing function used by projectMinimizer to print error
     * 
     * @param p Position
     * @param q Contact points
     */
    void
    projectMinimizerPrintInfo(
        p_t const & p
    ){
        printf("Guess: \n");
        for(int i = 0; i < 6*N; ++i)
            printf("%.5f, ", p(0));
        printf("\n");
        cextra_t _c = get_cextra(p);
        std::cout << "Error = " << _c.norm() << std::endl;
    }

    /*************************************************************************/
    /* GSL minimization solver: this one is based on solving a               */
    /* differential equation                                                 */
    /*************************************************************************/

    /**
     * @brief Find position and velocity that satisfies constraints by using a differential equation solver
     * 
     * @param p Initial guess for position
     * @param q Initial guess for contact points
     * @param v Initial guess for velocity
     * @return state_t Final state that satisfies constraints
     */
    state_t
    projectSolver(
        p_t const & p = p_t::Zero(),
        v_t const & v = v_t::Zero()
    )
    {

        p_t pf = p;
        projectSolverEvolve(pf);
        v_t vf = getProjectedVelocity(pf, v);

        state_t out;
        out.segment(0, 6*N) = pf;
        out.segment(6*N, 3*N) = vf;

        return out;
    }

    /**
     * @brief Method that invokes the differential equation solver that steers d/dt c(p) towards zero
     * 
     * @param p0 Initial guess for the position
     * @param time_step Time step used in solver
     * @param steps Maxium number of times steps
     * @param threshold Threshold used to terminate solver if it is reached from below
     * @return int Success or failure
     */
    int projectSolverEvolve(
        p_t & p0,
        double time_step = 0.01,
        int steps = 300,
        double threshold = 0.001
    ){
        //   gsl_odeiv2_system sys = {function, jacobian, dimension, pointer to parameters};
        gsl_odeiv2_system sys = {MS2DHOLSolverImpl<N, M1>::getPositionSolver, NULL, 6*N, this};

        gsl_odeiv2_driver * d = gsl_odeiv2_driver_alloc_y_new(
            &sys,
            gsl_odeiv2_step_rk8pd,
            1e-6,
            1e-6,
            0.0
        );

        printf("Initial:\n");
        projectSolverEvolvePrint(p0);

        double t = 0;
        double y[6*N];
        for(int i = 0; i < 6*N; ++i) y[i] = p0(i);

        for (int i = 0; i < steps; i++)
        {

            double ti = (i + 1) * time_step;
            int status = gsl_odeiv2_driver_apply(d, &t, ti, y);

            if (status != GSL_SUCCESS)
            {
                printf ("error, return value=%d\n", status);
                break;
            }
            if(projectSolverEvolveGetError(y) < threshold){
                printf("Solver got error below threshold after %d steps.\n", i);
                break;
            }
        }

        for(int i = 0; i < 6*N; ++i)  p0(i) = y[i];

        printf("Final:\n");
        projectSolverEvolvePrint(p0);

        gsl_odeiv2_driver_free(d);
        return 0;
    };

    /**
     * @brief Auxiliary method used by projectSolverEvolve
     * 
     * @param y Array with position p and contact points q
     * @return double Error ||c(p, q)||
     */
    double projectSolverEvolveGetError(
        double y[6*N]
    ){
        p_t p(&y[0]);
        cextra_t _cextra = get_cextra(p0);
        return _cextra.norm();
    }

    /**
     * @brief Auxiliary method used by projectSolverEvolve that prints position p and error c(p)
     * 
     * @param p0 Position p
     */
    void
    projectSolverEvolvePrint(
        p_t const & p0
    ){
        printf("Guess: \n");
        for(int i = 0; i < 6*N; ++i)
            printf("%.5f, ", p0(i));
        printf("\n");
        cextra_t _cextra = get_cextra(p0);
        std::cout << "Error = " << _cextra.norm() << std::endl;
    }


    /**
     * @brief Differential equation that guides p such that c(p) -> constant (hopefully zero)
     * 
     * @param t Time t
     * @param y Vector comprising of p
     * @param ddty Time derivative of p
     * @param params Pointer to object of this class
     * @return int Success or failure
     */
    static
    int
    getPositionSolver(
        double t,
        const double y[],
        double ddty[],
        void *params
    ){

        (void)(t); /* avoid unused parameter warning */
        // cast to pointer to object of this class
        MS2DHOLSolverImpl<N, M1> *ptr = (MS2DHOLSolverImpl<N, M1> *) params;

        p_t p(y);

        // add stability terms
        // for stability terms, we must consider the non holonomic constraints
        c_t _c = ptr->get_c(p);

        d1ck_t A = ptr->get_d1ck(p);
        v_t v_stability = -A.transpose()*(A*A.transpose()).llt().solve(_c);

        ddtp_t p_dot = ptr->ddtP(p, v_stability);

        // does not work
        // ddty = p_dot.data();

        // saves derivative to ddty
        for(int i = 0; i < 6*N; ++i)
            ddty[i] = p_dot(i);

        return GSL_SUCCESS;
    }

    /*************************************************************************/
    /* GSL differential equation solver                                      */
    /*************************************************************************/

    /**
     * @brief Function used by gsl_odeiv2_system that encapsulates d/dt state
     * 
     * @param t Time instant t
     * @param y State
     * @param f d/dt State
     * @param params Pointer to object on this class
     * @return int Success or failure
     */
    static
    int
    ddtState(
        double t,
        const double y[],
        double f[],
        void *params
    )
    {
        (void)(t); /* avoid unused parameter warning */
        MS2DHOLSolverImpl<N, M1> *p = (MS2DHOLSolverImpl<N, M1> *) params;

        /* this does not work:
        p->getStateDot(t, state_t(y)) is a temporary object
        and the operation 'f = p->getStateDot(t, state_t(y)).data()' does not copy anything
        so f will be pointing to just zeros,
        which is the default initialization
        */
        // f = p->getStateDot(state_t(y)).data();

        state_t x = p->getStateDot(t, state_t(y));
        for(int i = 0; i < (6+3)*N; ++i)
            f[i] = x(i);

        return GSL_SUCCESS;
    }

    /**
     * @brief Solve differential equations that represent dynamics of mechanical system
     * 
     * @param time_initial Initial time instant
     * @param time_step Time step
     * @param steps Number of steps
     * @param yall Where solution of differential equations is stored
     * @param current_step Indicates at which step we are currently at
     * @return int Success or failure
     */
    virtual
    int solverImplementation(
        double time_initial,
        double time_step,
        int steps,
        double** yall,
        int * current_step
    ){
        //   gsl_odeiv2_system sys = {function, jacobian, dimension, pointer to parameters};
        gsl_odeiv2_system sys = {MS2DHOLSolverImpl<N, M1>::ddtState, NULL, (6+3)*N, this};

        /*Explicit embedded Runge-Kutta Prince-Dormand (8, 9) method*/
        gsl_odeiv2_driver * d = gsl_odeiv2_driver_alloc_y_new(
            &sys,
            gsl_odeiv2_step_rk8pd,
            0.0005, // initial step size
            1e-6, // epsabs
            0.0 // epsrel
        );
        // set a minimum for allowed step size hmin for driver d
        // 0.5 milliseconds
        gsl_odeiv2_driver_set_hmin(d, 0.0005);

        double t = time_initial;
        double t1 = time_initial + steps*time_step;

        double y[(6+3)*N];
        // this does not work that well
        // state_t x0 = projectMinimizer(p0 v0);
        // this works better
        state_t x0 = projectSolver(p0, v0);

        for(int i = 0; i < (6+3)*N; ++i)
            y[i] = x0(i);

        for(int j = 0; j < (6+3)*N; ++j)
            yall[0][j] = y[j];

        for (int i = 0; i < steps; i++)
        {
            *current_step = i;
            // c_t _c = get_c(p_t(&y[0]));

            double ti = time_initial + (i + 1) * time_step;
            int status = gsl_odeiv2_driver_apply(d, &t, ti, y);
            f_t forces = getInternalForces(ti, state_t(y));

            for(int j = 0; j < (6+3)*N; ++j)
                yall[i][j] = y[j];
            for(int j = 0; j < 3*N; ++j)
                yall[i][(6+3)*N + j] = forces(j);

            if (status != GSL_SUCCESS)
            {
                printf ("error, return value=%d\n", status);
                // @todo throw exception here
                break;
            }

        }

        gsl_odeiv2_driver_free(d);
        return 0;
    };

    // @todo
    // virtual
    // void
    // initialStateParametizer(
    //     pdof_t /*parameter*/,
    //     velocity_t v = velocity_t::Zero
    // ){
    //     p0 = p_t::Zero();
    //     v0 = v;
    // }

    p_t p0;
    v_t v0;

private:
    /**
     * @brief position
     * 
     */
    std::string mName;

    /**
     * @brief Rigid body names
     * 
     */
    std::array<std::string, N> mRBNames;

    /**
     * @brief Inertia matrix
     * 
     */
    Eigen::Matrix<double, 3*N, 3*N> mI;
    /**
     * @brief Inverse of inertia matrix
     * 
     */
    Eigen::Matrix<double, 3*N, 3*N> mIinv;
    /**
     * @brief List with rigid bodies id's which have non-sliding contact constraints
     * 
     */
    std::list<int> mListWithContact;

    /**
     * @brief Linear and angular factors for plotting linear and angular velocity
     * 
     */
    std::array<double, 2> mVelocityPlotFactor;

    /**
     * @brief Linear and angular factors for plotting linear and angular force
     * 
     */
    std::array<double, 2> mForcePlotFactor;

    /**
     * @brief Linear and angular factors for plotting linear and angular force
     * 
     */
    std::array<double, 2> mGravity;

    /**
     * @brief Validate rigid body id and throw exception if it is not valid
     * 
     * @param i rigid body id
     */
    void validateId (
        int i
    ) const {
        if(i < 0 || i >= N){
            std::string msg = std::string("Mechanical system ") + mName + std::string(" has only ") + std::to_string(N) + std::string(" rigid bodies.");
            throw std::invalid_argument(msg);
        }
    }
};




