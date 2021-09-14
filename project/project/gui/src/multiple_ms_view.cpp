
#include "private/mainwindow.h"

// std::cout, std::endl
#include <iostream>

AdaptableQGraphicsView::AdaptableQGraphicsView(
    QGraphicsScene *scene,
    double scale,
    QWidget *parent
):
QGraphicsView(scene, parent),
mScale(scale > 1 ? scale : 1.05),
mScaleInv(scale > 1 ? 1/scale : 1/1.05)
{

}

void
AdaptableQGraphicsView::wheelEvent(QWheelEvent *ev){
    if(ev->angleDelta().y() > 0) // up Wheel
        scale(mScale, mScale);
    else if(ev->angleDelta().y() < 0) //down Wheel
        scale(mScaleInv, mScaleInv);
}

// void
// AdaptableQGraphicsView::mouseMoveEvent(QMouseEvent *ev){
//     QPointF p = ev->localPos();
//     // pos
//     // translate(p.rx(), p.ry());
//     // translate(1, 0);
//     // rotate(1);

//     // rotate(0);
//     // Because of the scene alignment (setAligment()), translating the view will have no visual impact.
//     // translate(0, 0);
//     // centerOn(0,0);
//     static int i = 0;
//     ++i;
//     setSceneRect(0 - i, 0, 200 - i, 200);
//     // fitInView(0, 0, 100, 100);
//     // centerOn(p.rx(), p.ry());
//     // scrollContentsBy(10, 0);

// }


MultipleMSView::MultipleMSView(
    YAML::Node const & node,
    QWidget * parent
):
QWidget(parent)
,
mNode(node)
,
mQGridLayout()
,
mQGraphicsScene()
,
// mQGraphicsView(&mQGraphicsScene, mNode["scale"].as<double, double>(1.05))
mQGraphicsView(&mQGraphicsScene, mNode["scale"].as<double>())
,
mTimerSlider(0)
,
mGlobalOptions(new GlobalOptions(mNode["GlobalOptions"]))
,
mQScrollArea(new QScrollArea())
,
mQTabWidget()
,
mList(std::vector<std::tuple<IMS2D*, QScrollArea*, MSOptions*>>({}))
,
mSolversUpdateTimer()
{
    // mQGraphicsScene.setSceneRect(-100, -100, 200, 200);
    mQGraphicsView.setMinimumWidth(700);
    mQGraphicsView.setMinimumHeight(500);
    mQGraphicsView.scale(1,-1);
    mQGraphicsView.setScene(&mQGraphicsScene);


    // QTabWidget

    // The widget becomes a child of the scroll area
    // and will be destroyed when the scroll area is deleted or when a new widget is set.
    // delete unncessary

    /*
    The widget becomes a child of the scroll area
    and will be destroyed when the scroll area is deleted or when a new widget is set
    */
    mQScrollArea->setWidget(mGlobalOptions);
    connect(
        mGlobalOptions, &GlobalOptions::startSolver,
        this, &MultipleMSView::solve
    );
    connect(
        mGlobalOptions, &GlobalOptions::changeGravity,
        this, &MultipleMSView::setGravity
    );
    connect(
        mGlobalOptions, &GlobalOptions::plotFactorsChanged,
        this, &MultipleMSView::plotFactorsChanged
    );
    /*
    addTab(QWidget *page, const QString &label)
    Adds a tab with the given page and label to the tab widget
    and returns the index of the tab in the tab bar
    Ownership of page is passed on to the QTabWidget.
    */
    mQTabWidget.addTab(mQScrollArea, QString("Global options"));
    mQTabWidget.setMinimumWidth(300);
    mQTabWidget.setTabsClosable(true);
    connect(
        &mQTabWidget, &QTabWidget::tabCloseRequested,
        this, &MultipleMSView::removeMS
    );

    mQGridLayout.addWidget(&mQGraphicsView, 0, 0, 1, 1);
    mQGridLayout.addWidget(&mTimerSlider, 1, 0, 1, 1);
    mQGridLayout.addWidget(&mQTabWidget, 0, 1, 1, 2);
    // The stretch factor is relative to the other columns in this grid. Columns with a higher stretch factor take more of the available space.
    mQGridLayout.setColumnStretch(0, 2);
    setLayout(&mQGridLayout);

    connect(
        &mTimerSlider, &TimerSlider::sliderMoved,
        this, &MultipleMSView::changeTime
    );

    connect(
        &mSolversUpdateTimer, &SolversUpdateTimer::setProgress,
        &mTimerSlider, &TimerSlider::setProgressValue
    );
}

MultipleMSView::~MultipleMSView()
{
    // deallocating memory done by scrollare and tabwidget
    // const int i_max = mList.size();
    // for(int i = 0; i < i_max; ++i){
    //     // delete QScrollArea
    //     delete std::get<1>(mList[0]);
    //     // delete MSOptions
    //     delete std::get<2>(mList[0]);
    //     mList.erase(mList.begin());
    // }
    std::cout << "Deleting multiple mechanical system view object." << std::endl;
}


YAML::Node
MultipleMSView::getNode() {
    mNode["GlobalOptions"] = mGlobalOptions->getNode();
    return mNode;
}

void
MultipleMSView::changeTime(
    int step
){
    for(std::size_t i = 0; i < mList.size(); ++i){
        std::get<0>(mList[i])->showAtStep(step);
        std::get<2>(mList[i])->set(step);
    }
    // mQGraphicsScene.setSceneRect(-100, -100, 200, 200);
}

void
MultipleMSView::addMS(
    IMS2D* ptr
){

    MSOptions* options = new MSOptions(ptr);
    QScrollArea * scroll = new QScrollArea();
    mList.push_back(
        std::make_tuple(ptr, scroll, options)
    );
    scroll->setWidget(options);
    mQTabWidget.addTab(scroll, QString(ptr->getName().c_str()));

    mQGraphicsScene.addItem(ptr->getView());

}


void MultipleMSView::removeMS(
    int index
){
    if(index==0)
        return;

    // delete scroll and options from above
    QWidget * q = mQTabWidget.widget(index);
    mQTabWidget.removeTab(index);
    delete q;
    

    // decremet to get mechanical system index
    index -= 1;
    IMS2D* ptr = std::get<0>(mList[index]);
    mQGraphicsScene.removeItem(ptr->getView());
    delete ptr;

    // remove element from the list
    mList.erase(mList.begin() + index);
}

void
MultipleMSView::solve(
    GlobalOptions::solver_options_t const & solver_options
){

    std::vector<IMS2D*> listIMS2D;
    for(std::size_t i = 0; i < mList.size(); ++i){
        listIMS2D.push_back(std::get<0>(mList[i]));
    }

    mTimerSlider.setSteps(
        GlobalOptions::getSteps(solver_options)
    );
    mSolversUpdateTimer.startSolvers(
        listIMS2D,
        solver_options
    );

    /*
    older approach which was not ok
    for(std::size_t i = 0; i < mList.size(); ++i){

        IMS2D* ptr = std::get<0>(mList[i]);
        WorkerThread *workerThread = new WorkerThread(
            ptr,
            GlobalOptions::getInitialTime(solver_options),
            GlobalOptions::getDeltaTime(solver_options),
            GlobalOptions::getSteps(solver_options)
        );

        // connect(workerThread, &QThread::finished, workerThread, &QObject::deleteLater);

        // this approach does not work when we have several solvers
        // working at the same time
        // connect(workerThread, &WorkerThread::setProgress, &mTimerSlider, &TimerSlider::setProgressValue);

        workerThread->start();
    }

    */
}

void MultipleMSView::plotFactorsChanged(
    IMS2D::plot_factors_t plot_factors
){
    for(std::size_t i = 0; i < mList.size(); ++i){
        IMS2D* p = std::get<0>(mList[i]);
        p->setPlotFactors(plot_factors);
    }
}

void MultipleMSView::setGravity(
    std::array<double, 2> const & gravity
){
    for(std::size_t i = 0; i < mList.size(); ++i){
        IMS2D* p = std::get<0>(mList[i]);
        p->setGravity(gravity);
    }
}