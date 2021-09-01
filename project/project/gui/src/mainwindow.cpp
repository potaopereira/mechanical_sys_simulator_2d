
#include "private/mainwindow.h"

// std::cout, std::endl
#include <iostream>

#include <QSize>

#include <QPen>

#include <iostream>
#include <chrono>
#include <thread>

#include <QPolygonF>
#include <QPainterPath>

#include <math.h>

// std::ofstream
#include <fstream>

// std::cout, std::endl
#include <iostream>

SingleRBViewView::SingleRBViewView(
    QObject * parent = 0
):
QObject(parent)
,
mQRectF()
,
mQGraphicsPathItem(new QGraphicsPathItem())
,
mQGraphicsRectItem(new QGraphicsRectItem())
,
mQGraphicsEllipseItem(new QGraphicsEllipseItem())
,
mE1(new QGraphicsLineItem())
,
mE2(new QGraphicsLineItem())
,
mQGraphicsItemGroup()
,
mEI1(new QGraphicsLineItem())
,
mEI2(new QGraphicsLineItem())
,
q1(new QGraphicsLineItem())
,
q2(new QGraphicsLineItem())
,
lf(new QGraphicsLineItem())
,
af(new QGraphicsLineItem())
{

    QPolygonF myPolygon;
    int k = 150;
    for(int i = 0; i < k + 1; ++i){
        double a = 1;
        double b = 1.001;
        double theta = i*2*M_PI/k;
        double c = cos(2*theta);
        double r = sqrt(pow(a, 2)*c + sqrt(pow(b, 4) + pow(a, 4)*(pow(c, 2) - 1) ));
        myPolygon << QPointF(100*r*cos(theta), 100*r*sin(theta));
    }

    QPainterPath* myPath = new QPainterPath();
    myPath->addPolygon(myPolygon);
    // myPath.setPen(myPen);
    mQGraphicsPathItem->setPath(*myPath);
    mQGraphicsItemGroup.addToGroup(mQGraphicsPathItem);

    mQRectF.setSize(QSizeF(100, 100));
    // setSize needs to be set first
    mQRectF.moveCenter(QPointF(100, 100));

    mQGraphicsEllipseItem->setRect(mQRectF);
    mQGraphicsEllipseItem->setRotation(0);
    mQGraphicsItemGroup.addToGroup(mQGraphicsEllipseItem);

    mQGraphicsRectItem->setRect(-100, -100, +200, +200);
    mQGraphicsItemGroup.addToGroup(mQGraphicsRectItem);




    // red: Hex: #FF0000 RGBA(255, 0, 0, 1)
    QPen pen1(Qt::red, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    mE1->setPen(pen1);
    mE1->setLine(100, 100, 100 + 20, 100);
    mQGraphicsItemGroup.addToGroup(mE1);

    // QPen pen2(Qt::green, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    // QColor(int r, int g, int b, int a = 255);
    // dark green: Hex: #006400 RGBA(0, 100, 0, 1)
    QPen pen2(QColor(0, 100, 0), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    mE2->setPen(pen2);
    mE2->setLine(100, 100, 100, 100 + 20);
    mQGraphicsItemGroup.addToGroup(mE2);


    mEI1->setPen(pen1);
    mEI1->setLine(0, 0, 10, 0);
    mQGraphicsItemGroup.addToGroup(mEI1);

    mEI2->setPen(pen2);
    mEI2->setLine(0, 0, 0, 10);
    mQGraphicsItemGroup.addToGroup(mEI2);

    mQGraphicsItemGroup.addToGroup(q1);
    mQGraphicsItemGroup.addToGroup(q2);

    // dark brown: Hex: #5C4033: RGBA(92, 64, 51, 1)
    QPen penf(QColor(92, 64, 51), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    lf->setPen(penf);
    mQGraphicsItemGroup.addToGroup(lf);
    // dark grey: Hex: #A9A9A9, RGBA(169, 169, 169, 1)
    QPen penaf(QColor(169, 169, 169), 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    af->setPen(penaf);
    mQGraphicsItemGroup.addToGroup(af);
}

void
SingleRBViewView::change(
    int p1,
    int p2,
    double r00,
    double r01,
    double r10,
    double r11,
    double lf1,
    double lf2,
    double aff
)
{
    mQRectF.moveCenter(QPointF(p1, p2));
    mQGraphicsEllipseItem->setRect(mQRectF);

    mE1->setLine(p1, p2, p1 + 20 * r00, p2 + 20 * r10);
    mE2->setLine(p1, p2, p1 + 20 * r01, p2 + 20 * r11);

    {
    double x1 = p1 - 1 *r00*100;
    double x2 = p2 - 1 *r10*100;
    q1->setLine(x1, x2, 0, 0);
    }

    {
    double x1 = p1 + 1 *r00*100;
    double x2 = p2 + 1 *r10*100;
    q2->setLine(x1, x2, 2.2*100, 0);
    }

    lf->setLine(p1, p2, p1 + lf1, p2 + lf2);
    af->setLine(p1, p2, p1 + aff*(r00 + r01)*100, p2 + aff*(r10 + r11)*100);


}

TimerSlider::TimerSlider(
    int steps,
    QWidget * parent
):
QWidget(parent)
,
mSteps(steps)
,
mMaxCurrent(0)
,
mQGridLayout()
,
mTInitial(QString("time=0"))
,
mTEnd(QString("time=1"))
,
mQSlider(Qt::Horizontal)
,
mQProgressBar()
,
mQPushButtonNext(QString("Next"))
,
mQPushButtonPrevious(QString("Previous"))
{

    mQProgressBar.setMinimum(0);
    mQProgressBar.setMaximum(mSteps);

    mQSlider.setMinimumWidth(50);
    mQSlider.setMaximumHeight(15);
    mQSlider.setToolTip(QString("max_hour_help"));
    mQSlider.setSingleStep(1);
    mQSlider.setMaximum(mSteps);
    mQSlider.setMinimum(0);
    mQSlider.setValue(0);
    mQSlider.setEnabled(false);

    mTInitial.setMaximumHeight(15);
    mQSlider.setMaximumHeight(15);
    mTEnd.setMaximumHeight(15);

    mQGridLayout.addWidget(&mTInitial, 0, 0);
    mQGridLayout.addWidget(&mQSlider, 0, 1);
    mQGridLayout.addWidget(&mTEnd, 0, 2);
    mQGridLayout.addWidget(&mQPushButtonPrevious, 1, 0);
    mQGridLayout.addWidget(&mQProgressBar, 1, 1);
    mQGridLayout.addWidget(&mQPushButtonNext, 1, 2);

    setLayout(&mQGridLayout);

    connect(
        &mQSlider, &QSlider::sliderMoved,
        this, &TimerSlider::sliderMoved
    );
    connect(
        &mQPushButtonNext, &QPushButton::pressed,
        this, &TimerSlider::goNext
    );
    connect(
        &mQPushButtonPrevious, &QPushButton::pressed,
        this, &TimerSlider::goPrevious
    );

}

void
TimerSlider::setSteps(
    int unsigned steps
){
    mSteps = steps;

    mQProgressBar.setValue(0);
    mQProgressBar.setMaximum(mSteps);

    mQSlider.setValue(0);
    mQSlider.setMaximum(mSteps);
}


void
TimerSlider::setValue(
    int value
){
    if(value <= mMaxCurrent)
        emit sliderMoved(value);
}

void
TimerSlider::goNext(
){
    int k = mQSlider.value();
    ++k;
    if(k <= mSteps){
        mQSlider.setValue(k);
        emit sliderMoved(k);
    }
}

void
TimerSlider::goPrevious(
){
    int k = mQSlider.value();
    --k;
    if(k >= 0){
        mQSlider.setValue(k);
        emit sliderMoved(k);
    }
}

TimerSlider::~TimerSlider(

){
    std::cout << "Deleting timer slider." << std::endl;
}

void
TimerSlider::setProgressValue(
    int value
){
    mMaxCurrent = value;
    mQProgressBar.setValue(mMaxCurrent);
    // if(value==mSteps){
    //     mQSlider.setEnabled(true);
    // }
    mQSlider.setEnabled(true);
}

WorkerThread::WorkerThread(
    IMS2D* p,
    double time_initial,
    double time_step,
    int steps,
    int unsigned timersleep_ms
):
m(p),
mTimeInitial(time_initial),
mTimeStep(time_step),
mSteps(steps),
mCurrentStep(0),
mQTimer()
{
    connect(
        &mQTimer, &QTimer::timeout,
        this, &WorkerThread::emitProgress
    );

    mQTimer.start(timersleep_ms);
}

void
WorkerThread::run() {

    // doesn't work here
    // mQTimer.start(500);

    // int solve(
    //     double time_initial,
    //     double time_step,
    //     int steps,
    //     int * current_step
    // )

    m->solve(
        mTimeInitial,
        mTimeStep,
        mSteps + 1,
        &mCurrentStep
    );

    // Timers cannot be stopped from another thread
    // mQTimer.stop();

    emit setProgress(mCurrentStep);
    emit resultReady(QString("result ready"));
}

void
WorkerThread::emitProgress() {
    emit setProgress(mCurrentStep);
}

int
WorkerThread::getProgrees() const{
    return mCurrentStep;
}


SolversUpdateTimer::SolversUpdateTimer(
    int unsigned timersleep_ms,
    QObject *parent
):
QTimer(parent),
mTimerMs(timersleep_ms),
mThreadsList(std::vector<WorkerThread*>({})),
mFinished(true),
mFinalProgess(0)
{
    setInterval(timersleep_ms);

    connect(
        this, &QTimer::timeout,
        this, &SolversUpdateTimer::solverupdate
    );

}

bool
SolversUpdateTimer::startSolvers(
    std::vector<IMS2D*> list,
    GlobalOptions::solver_options_t solver_options
)
{
    if(!mFinished){
        return false;
    }
    else{
        // nothing to be done if list is empty
        if(list.size() == 0)
            return true;

        mFinalProgess = GlobalOptions::getSteps(solver_options);

        for(std::size_t i = 0; i < list.size(); ++i){

            IMS2D* ptr = list[i];
            WorkerThread *workerThread = new WorkerThread(
                ptr,
                GlobalOptions::getInitialTime(solver_options),
                GlobalOptions::getDeltaTime(solver_options),
                GlobalOptions::getSteps(solver_options),
                mTimerMs
            );
            mThreadsList.push_back(workerThread);

            // threads cannot be deleted when they come to an end
            // because access to them is required at SolversUpdateTimer::solverupdate
            // connect(
            //     workerThread, &QThread::finished,
            //     workerThread, &QObject::deleteLater
            // );

            workerThread->start();
        }

        // start timer
        start();
        return true;
    }

}

void
SolversUpdateTimer::solverupdate() {

    int progress = 0;
    int N = mThreadsList.size();
    for(int i = 0; i < N; ++i){
        progress += mThreadsList[i]->getProgrees();
    }
    progress = progress / N;
    emit setProgress(progress);

    if(progress == mFinalProgess){
        std::cout << "All solvers ended." << std::endl;
        // stop timer
        stop();
        // delete all solvers
        for(int i = 0; i < N; ++i){
            // thread should have finished, but we need to wait for run to have come to an end
            mThreadsList[0]->wait();

            delete mThreadsList[0];
            // clear vector
            mThreadsList.erase(mThreadsList.begin());
        }
    }
}

RBViewView::RBViewView(
    int steps,
    QWidget * parent
):
QWidget(parent)
,
mQVBoxLayout()
,
mQGraphicsScene()
,
mQGraphicsView(&mQGraphicsScene)
,
mSteps(steps)
,
mTimerSlider(mSteps)
,
mGlobalOptions(new GlobalOptions())
,
mQScrollArea(new QScrollArea())
,
mQTabWidget()
,
mList(std::vector<std::tuple<IMS2D*, QScrollArea*, MSOptions*>>({}))
,
mSolversUpdateTimer()
{
    mQGraphicsScene.setSceneRect(-100, -100, 200, 200);
    mQGraphicsView.setMinimumWidth(500);
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
        this, &RBViewView::solve
    );
    /*
    addTab(QWidget *page, const QString &label)
    Adds a tab with the given page and label to the tab widget
    and returns the index of the tab in the tab bar
    Ownership of page is passed on to the QTabWidget.
    */
    mQTabWidget.addTab(mQScrollArea, QString("Global options"));
    mQTabWidget.setMinimumHeight(200);

    mQVBoxLayout.addWidget(&mQGraphicsView);
    mQVBoxLayout.addWidget(&mTimerSlider);
    mQVBoxLayout.addWidget(&mQTabWidget);
    setLayout(&mQVBoxLayout);

    connect(
        &mTimerSlider, &TimerSlider::sliderMoved,
        this, &RBViewView::changeTime
    );

    connect(
        &mSolversUpdateTimer, &SolversUpdateTimer::setProgress,
        &mTimerSlider, &TimerSlider::setProgressValue
    );
}

RBViewView::~RBViewView()
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

    std::cout << "Deleting rigid body view object." << std::endl;
}

void
RBViewView::changeTime(
    int step
){
    // mSingleRBViewView.change(y[value][0]*100, y[value][1]*100, y[value][2], y[value][3], y[value][4], y[value][5], y[value][9], y[value][10], y[value][11]);
    // pp->setPose(
    //     Eigen::Matrix<double, (2 + 4), 1>(&(y[value][0]))
    //     ,
    //     Eigen::Matrix<double, (3), 1>(&(y[value][2+4+2]))
    // );

    for(std::size_t i = 0; i < mList.size(); ++i){
        std::get<0>(mList[i])->showAtStep(step);
        std::get<2>(mList[i])->set(step);
    }
    // mQGraphicsScene.setSceneRect(-100, -100, 200, 200);
}

void
RBViewView::addMS(
    IMS2D* ptr
){

    MSOptions* options = new MSOptions(ptr);
    QScrollArea * scroll = new QScrollArea();
    mList.push_back(
        std::make_tuple(ptr, scroll, options)
    );
    scroll->setWidget(options);
    mQTabWidget.addTab(scroll, QString("a"));

    mQGraphicsScene.addItem(ptr->getView());

}

void
RBViewView::solve(
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

MainWindow::MainWindow(
    QWidget *parent
):
QMainWindow(parent)
,
mNode(getConfig())
,
mAddMS()
,
mRBViewView()
// ,
// mQMenuBar(tr("My Menu"))
{

    QAction* newAct = new QAction(tr("&New"), this);
    newAct->setShortcuts(QKeySequence::New);
    newAct->setStatusTip(tr("Create a new file"));
    connect(newAct, &QAction::triggered, this, &MainWindow::newFile);

    // You should then add some actions to that menu.
    //  Then you want to add the menu to the tool bar:
    QMenu * fileMenu = menuBar()->addMenu(tr("&Rigid bodies"));
    fileMenu->addAction(newAct);

    QAction* boldAct = new QAction(tr("&New"), this);
    // newAct->setShortcuts(QKeySequence::New);
    // newAct->setStatusTip(tr("Create a new file"));
    connect(boldAct, &QAction::triggered, this, &MainWindow::newFile);


    QMenu * formatMenu = fileMenu->addMenu(tr("&Format"));
    formatMenu->addAction(boldAct);

    
    fileMenu->addMenu(formatMenu);

    fileMenu->addMenu(&mAddMS);
    connect(
        &mAddMS, &AddMS::addedMS,
        this, &MainWindow::addMS
    );
    // theTargetToolBar->addAction(mQMenuBar.menuAction());
    // You probably want to have some sort of icon:
    // mQMenuBar.menuAction()->setIcon(myIcon);

    setGeometry(400, 250, 542, 390);
    setCentralWidget(&mRBViewView);
}

YAML::Node
MainWindow::getConfig(){
    try{
        YAML::Node node = YAML::LoadFile("config_tmp.yaml");
        return node;
    }
    catch(...){
        std::cout << "Using default config." << std::endl;
        return YAML::LoadFile("config.yaml");
    }
    
}

MainWindow::~MainWindow()
{
    std::ofstream config_file("config_tmp.yaml");
    config_file << mNode;
}

void
MainWindow::newFile(
    bool
)
{

}

void
MainWindow::addMS(
    IMS2D* ms_ptr
)
{
    mRBViewView.addMS(ms_ptr);
}
