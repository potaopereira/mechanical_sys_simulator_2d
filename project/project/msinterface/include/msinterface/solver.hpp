#pragma once

// std::vector
#include <vector>

#include <Eigen/Dense>

class IMS2DSolver {
public:


    /**
     * @brief rigid body position
     * 
     */
    typedef Eigen::Matrix<double, 6, 1> rbp_t;
    /**
     * @brief rigid body velocity
     * 
     */
    typedef Eigen::Matrix<double, 3, 1> rbv_t;
    /**
     * @brief rigid body force
     * 
     */
    typedef Eigen::Matrix<double, 3, 1> rbf_t;
    /**
     * @brief vector
     * 
     */
    typedef Eigen::Matrix<double, 2, 1> v_t;


    /**
     * @brief Construct a new IMRB2D object
     * 
     */
    IMS2DSolver();

    virtual ~IMS2DSolver();

    /**************************************************************************/
    /* pure virtual methods                                                   */
    /**************************************************************************/

    virtual int dimSolution() const = 0;
    virtual int dimRigidBodies() const = 0;
    virtual int dimConstraints() const = 0;

    virtual
    int solverImplementation(
        double time_initial,
        double time_step,
        int steps,
        double** solution,
        int * current_step
    ) = 0;

    /**************************************************************************/
    /* for plotting I decided to use eigen                                    */
    /**************************************************************************/

    virtual
    rbp_t getPositionPlot(
        double* solution,
        int N // rigid body number
    ) const = 0;

    virtual
    rbv_t getVelocityPlot(
        double* solution,
        int N // rigid body number
    ) const = 0;

    virtual
    rbf_t getForcePlot(
        double* solution,
        int N // rigid body number
    ) const = 0;

    virtual
    std::vector<v_t> getInertialVectorsPlot(
        double* solution,
        int N // rigid body number
    ) const = 0;

    virtual
    std::vector<v_t> getBodyVectorsPlot(
        double* solution,
        int N // rigid body number
    ) const = 0;

    /**************************************************************************/
    /* for showing numbers  I decided to use arrays                           */
    /**************************************************************************/

    virtual
    std::array<double, 3> getPosition(
        double* solution,
        int N // rigid body number
    ) const = 0;

    virtual
    std::array<double, 3> getVelocity(
        double* solution,
        int N // rigid body number
    ) const = 0;

    virtual
    std::array<double, 3> getForce(
        double* solution,
        int N // rigid body number
    ) const = 0;

    virtual
    std::array<double, 2> getPositionOfPoint(
        double* solution,
        int N, // rigid body number
        std::array<double, 2> relative_position
    ) const = 0;

    virtual
    std::array<double, 2> getVelocityOfPoint(
        double* solution,
        int N, // rigid body number
        std::array<double, 2> relative_position
    ) const = 0;

    /**************************************************************************/
    /* basics of the interface                                                */
    /**************************************************************************/

    virtual
    int solve(
        double time_initial,
        double time_step,
        int steps,
        int * current_step
    );

    /**************************************************************************/
    /* for plotting I decided to use eigen                                    */
    /**************************************************************************/

    rbp_t getPositionPlot(
        int step,
        int N // rigid body number
    ) const;

    rbv_t getVelocityPlot(
        int step,
        int N // rigid body number
    ) const;

    rbf_t getForcePlot(
        int step,
        int N // rigid body number
    ) const;

    std::vector<v_t> getInertialVectorsPlot(
        int step,
        int N // rigid body number
    ) const;

    std::vector<v_t> getBodyVectorsPlot(
        int step,
        int N // rigid body number
    ) const;

    /**************************************************************************/
    /* for showing numbers  I decided to use arrays                           */
    /**************************************************************************/

    virtual
    std::array<double, 3> getPosition(
        int step,
        int N // rigid body number
    ) const;

    virtual
    std::array<double, 3> getVelocity(
        int step,
        int N // rigid body number
    ) const;

    virtual
    std::array<double, 3> getForce(
        int step,
        int N // rigid body number
    ) const;

    std::array<double, 2> getPositionOfPoint(
        int step,
        int N, // rigid body number
        std::array<double, 2> relative_position
    ) const;

    std::array<double, 2> getVelocityOfPoint(
        int step,
        int N, // rigid body number
        std::array<double, 2> relative_position
    ) const;

private:
    bool mAllocated;
    double** mSolutions;
    int mSteps;

    bool
    allocate(
        int unsigned steps
    );

    bool
    deallocate(
    );

};