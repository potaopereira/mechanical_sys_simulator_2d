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
    void showPosition(
        int body_id,
        bool show
    ){
        if(body_id < N){
            if(show)
                mListOfBodies[body_id]->showPosition();
            else
                mListOfBodies[body_id]->hidePosition();
        }
    }

    virtual
    void showVelocity(
        int body_id,
        bool show
    ){
        if(body_id < N){
            if(show)
                mListOfBodies[body_id]->showVelocity();
            else
                mListOfBodies[body_id]->hideVelocity();
        }
    }

    virtual
    void showLinearVelocity(
        int body_id,
        bool show
    ){
        if(body_id < N){
            if(show)
                mListOfBodies[body_id]->showLinearVelocity();
            else
                mListOfBodies[body_id]->hideLinearVelocity();
        }
    }

    virtual
    void showAngularVelocity(
        int body_id,
        bool show
    ){
        if(body_id < N){
            if(show)
                mListOfBodies[body_id]->showAngularVelocity();
            else
                mListOfBodies[body_id]->hideAngularVelocity();
        }
    }

    virtual
    void showForce(
        int body_id,
        bool show
    ){
        if(body_id < N){
            if(show)
                mListOfBodies[body_id]->showForce();
            else
                mListOfBodies[body_id]->hideForce();
        }
    }

    virtual
    void showLinearForce(
        int body_id,
        bool show
    ){
        if(body_id < N){
            if(show)
                mListOfBodies[body_id]->showLinearForce();
            else
                mListOfBodies[body_id]->hideLinearForce();
        }
    }

    virtual
    void showAngularForce(
        int body_id,
        bool show
    ){
        if(body_id < N){
            if(show)
                mListOfBodies[body_id]->showAngularForce();
            else
                mListOfBodies[body_id]->hideAngularForce();
        }
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
    void showRelativePoint(
        bool show,
        int body_id // rigid body number
    ){
        if(body_id < N)
            mListOfBodies[body_id]->showRelativePoint(show);
    }

    virtual
    void setRelativePoint(
        int body_id, // rigid body number
        std::array<double, 2> point
    ){
        mListOfBodies[body_id]->setRelativePoint(point);
    }

    virtual
    void showRelativePointVelocity(
        bool show,
        int body_id
    ){
        if(body_id < N)
            mListOfBodies[body_id]->showVectorAtPoint(show);
    }

    virtual
    void setRelativePointVelocity(
        int body_id, // rigid body number
        std::array<double, 2> point,
        std::array<double, 2> velocity
    ){
        if(body_id < N)
            mListOfBodies[body_id]->setVectorAtPoint(point, velocity);
    }

    virtual
    void showRelativePointPath(
        bool show,
        int body_id, // rigid body number
        int unsigned length
    ){
        if(body_id < N)
            mListOfBodies[body_id]->showPointPath(show, length);
    }

    virtual
    void setRelativePointPath(
        int body_id, // rigid body number
        std::array<double, 2> point
    ){
        if(body_id < N)
            mListOfBodies[body_id]->addToPointPath(point);
    }

    virtual
    QGraphicsItemGroup* getView() {
        return this;
    };


private:
    std::array<RBViewWVectors*, N> mListOfBodies;
    QGraphicsItemGroup mConstraints;
};