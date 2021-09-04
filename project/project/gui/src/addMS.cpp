#include <private/addMS.h>

// // ExampleMSViewer
// #include <ExampleMS/ExampleMSViewer.hpp>

// BallOnSlopeViewer
#include <BallOnSlope/BallOnSlopeViewer.hpp>

// SlidingBallOnSlopeViewer
#include <SlidingBallOnSlope/SlidingBallOnSlopeViewer.hpp>

// BallOnCircleViewer
#include <BallOnCircle/BallOnCircleViewer.hpp>

// SlidingBallOnCircleViewer
#include <SlidingBallOnCircle/SlidingBallOnCircleViewer.hpp>

// BarPendulumViewer
#include <BarPendulum/BarPendulumViewer.hpp>

IMS2D*
MSFactory::create(
    MSType type
){
    switch(type){
        case(MSType::BallOnSlopeT):
            return new BallOnSlopeViewer();
        case(MSType::SlidingBallOnSlopeT):
            return new SlidingBallOnSlopeViewer();

        case(MSType::BallOnCircleT):
            return new BallOnCircleViewer();
        case(MSType::SlidingBallOnCircleT):
            return new SlidingBallOnCircleViewer();

        case(MSType::BarPendulumT):
            return new BarPendulumViewer();
        
        // case(MSType::ExampleMST):
        //     return new ExampleMSViewer();
        // case(MSType::PendulumT):
        //     return new Pendulum();
        // case(MSType::BallOnParabolaNoSlidingT):
        //     return new BallOnParabolaNoSliding();
        default:
            return nullptr;
    }
}

std::vector<std::pair<std::string, MSFactory::MSType>>
AddMS::mList = {
    make_pair(
        std::string("BallOnSlopeT"),
        MSFactory::MSType::BallOnSlopeT
    )
    ,
    make_pair(
        std::string("SlidingBallOnSlopeT"),
        MSFactory::MSType::SlidingBallOnSlopeT
    )
    ,
    make_pair(
        std::string("BallOnCircleT"),
        MSFactory::MSType::BallOnCircleT
    )
    ,
    make_pair(
        std::string("SlidingBallOnCircleT"),
        MSFactory::MSType::SlidingBallOnCircleT
    )
    ,
    make_pair(
        std::string("BarPendulumT"),
        MSFactory::MSType::BarPendulumT
    )
    // ,
    // make_pair(
    //     std::string("ExampleMST"),
    //     MSFactory::MSType::ExampleMST
    // )
    // make_pair(
    //     std::string("Pendulum"),
    //     MSFactory::MSType::PendulumT
    // )
    // ,
    // make_pair(
    //     std::string("BallOnParabolaNoSliding"),
    //     MSFactory::MSType::BallOnParabolaNoSlidingT
    // )
};

AddMS::AddMS(
    QWidget * parent
):
QMenu(
    QString("&Add Mechanical Sys."),
    parent
),
mQActionGroup(this)
{
    for(std::size_t i = 0; i < mList.size(); ++i){
        QAction * a = new QAction(
            QString::fromStdString(mList[i].first)
        );
        mActions.push_back(a);
        mMap[a] = mList[i].second;
        addAction(a);
        mQActionGroup.addAction(a);
    }
    setStatusTip(tr("Add a new Mechanical System"));

    connect(
        &mQActionGroup, &QActionGroup::triggered,
        this, &AddMS::add
    );
}

AddMS::~AddMS(){
    const std::size_t i_max = mActions.size();
    for(std::size_t i = 0; i < i_max; ++i){
        delete mActions[0];
        mActions.erase(mActions.begin());
    }
}

void
AddMS::add(
    QAction *action
){
    IMS2D * newMS = MSFactory::create(mMap[action]);
    emit addedMS(newMS);
}