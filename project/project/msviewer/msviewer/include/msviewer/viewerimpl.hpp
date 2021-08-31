#pragma once

#include "msviewer/rbviewwvectors.hpp"

// mechanical system
template<int N>
class MSView:
public QGraphicsItemGroup,
public IMS2DViewer
{
public:
    MSView(
        std::array<RBViewWVectors*, N> const & listOfBodies
    ):
    QGraphicsItemGroup()
    ,
    IMS2DViewer()
    ,
    mListOfBodies(listOfBodies)
    ,
    mConstraints()
    {

        for(int i = 0; i < N; ++i){
            addToGroup(mListOfBodies[i]);
        }
    }

    void reset(
        std::array<void*, N> params
    ){
        for(int i = 0; i < N; ++i){
            mListOfBodies[i]->resetBoundary(params[i]);
        }
    }

    void setConstraints(
        std::vector<QGraphicsItem*> const & listOfConstraints
    ){
        for(std::size_t i = 0; i < listOfConstraints.size(); ++i){
            mConstraints.addToGroup(listOfConstraints[i]);
            addToGroup(&mConstraints);
        }
    }

    void
    setPoseAndVectors(
        Eigen::Matrix<double, (2 + 4)*N, 1> position
        ,
        Eigen::Matrix<double, (2 + 1)*N, 1> velocity
        ,
        Eigen::Matrix<double, (2 + 1)*N, 1> force
        ,
        std::array<std::vector<Eigen::Matrix<double, 2, 1>>, N> const & inertial_vectors_list
        ,
        std::array<std::vector<Eigen::Matrix<double, 2, 1>>, N> const & body_vectors_list
    ){
        for(int i = 0; i < N; ++i){
            mListOfBodies[i]->setPoseAndVectors(
                position.segment(6*i, 6),
                velocity.segment(3*i, 3),
                force.segment(3*i, 3),
                inertial_vectors_list[i],
                body_vectors_list[i]
            );
        }
    }

    void
    setPose(
        Eigen::Matrix<double, (2 + 4)*N, 1> position
    ){
        for(int i = 0; i < N; ++i){
            mListOfBodies[i]->setPose(position.segment(6*i, 6));
        }
    }

    /**************************************************************************/
    /* implementing pure virtual methods  of IMS2DViewer                      */
    /**************************************************************************/

    virtual
    void setPoseAndVectorsOfRB(
        rbp_t const & position
        ,
        rbv_t velocity
        ,
        rbf_t force
        ,
        std::vector<v_t> const & inertial_vectors
        ,
        std::vector<v_t> const & body_vectors
        ,
        int body_id
    ){
        // if(0= < body_id < N)
        mListOfBodies[body_id]->setPoseAndVectors(
            position,
            velocity,
            force,
            inertial_vectors,
            body_vectors
        );
    };

    virtual
    void setPosition(
        rbp_t const & position
        ,
        int body_id
    ){
        mListOfBodies[body_id]->setPose(position);
    };

    virtual
    void setVelocity(
        rbv_t const & velocity
        ,
        int body_id
    ){
        mListOfBodies[body_id]->setVelocity(velocity);
    }

    virtual
    void setForce(
        rbf_t const & force
        ,
        int body_id
    ){
        mListOfBodies[body_id]->setForce(force);
    }

    virtual
    void showBoundary(
        int body_id,
        bool show
    ){
        if(body_id < N){
            mListOfBodies[body_id]->showBoundary(show);
        }
    }

    virtual
    void setBoundaryColor(
        int body_id,
        QColor color
    ){
        if(body_id < N){
            mListOfBodies[body_id]->setBoundaryColor(color);
        }
    }

    virtual
    QGraphicsItemGroup* getView() {
        return this;
    };


private:
    std::array<RBViewWVectors*, N> mListOfBodies;
    QGraphicsItemGroup mConstraints;
};