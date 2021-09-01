#pragma once

#include "private/colorbutton.h"
// MSOptions
#include "private/mechanical_system_options.h"
// GlobalOptions
#include "private/global_options.h"

// std::vector
#include <vector>
// std::pair
#include <utility>


// class AddMS
#include "private/addMS.h"

#include <QMainWindow>
#include <memory>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
// QSlider
#include <QSlider>

// QGraphicsScene
#include <QGraphicsScene>

// QGraphicsView
#include <QGraphicsView>

// QMenuBar
#include <QMenuBar>

// QAction
#include <QAction>

// QMenu
#include <QMenu>

// QScrollArea
#include <QScrollArea>

// QGridLayout
#include <QGridLayout>

// QLineEdit
#include <QLineEdit>

// QCheckBox
#include <QCheckBox>

// QPushButton
#include <QPushButton>

// QRectF
#include <QRectF>

// QGraphicsEllipseItem
#include <QGraphicsEllipseItem>

// QGraphicsRectItem
#include <QGraphicsRectItem>

// QGraphicsLineItem
#include <QGraphicsLineItem>

// QGraphicsPathItem
#include <QGraphicsPathItem>

// QGraphicsItemGroup
#include <QGraphicsItemGroup>

//
#include <QWheelEvent>

// QProgressBar
#include <QProgressBar>

// QTimer
#include <QTimer>

// QThread
#include <QThread>

// #include <pendulum/pendulum.hpp>

// #include <pendulum/ball_on_parabola.hpp>
// #include <pendulum/ball_on_parabola_no_sliding.hpp>

// RBViewRectangle
// #include <view/combined.hpp>

#include <yaml-cpp/yaml.h>

class SingleRBViewView: public QObject
{
    Q_OBJECT
public:
    SingleRBViewView(QObject * parent);
    QGraphicsItem* get(){
        return &mQGraphicsItemGroup;
    }
    void change(
        int p1,
        int p2,
        double r00,
        double r01,
        double r10,
        double r11,
        double lf1,
        double lf2,
        double af
    );
private:
    QRectF mQRectF;
    QGraphicsPathItem* mQGraphicsPathItem;
    QGraphicsRectItem* mQGraphicsRectItem;
    QGraphicsEllipseItem* mQGraphicsEllipseItem;
    QGraphicsLineItem* mE1;
    QGraphicsLineItem* mE2;
    QGraphicsItemGroup mQGraphicsItemGroup;

    QGraphicsLineItem* mEI1;
    QGraphicsLineItem* mEI2;

    QGraphicsLineItem* q1;
    QGraphicsLineItem* q2;

    QGraphicsLineItem* lf;
    QGraphicsLineItem* af;
};



class TimerSlider: public QWidget
{
    Q_OBJECT
public:
    TimerSlider(
        int steps = 0,
        QWidget *parent = 0
    );
    ~TimerSlider();

    void setSteps(
        int unsigned steps
    );

public slots:
    void setValue(
        int value
    );
    void
    setProgressValue(
        int value
    );
    void
    goNext(
    );
    void
    goPrevious(
    );
signals:
    void sliderMoved(
        int value
    );
private:
    int mSteps;
    int mMaxCurrent;
    QGridLayout mQGridLayout;
    QLabel mTInitial;
    QLabel mTEnd;
    QSlider mQSlider;
    QProgressBar mQProgressBar;
    QPushButton mQPushButtonNext;
    QPushButton mQPushButtonPrevious;
};

class PQGraphicsView: public QGraphicsView
{
public:
    PQGraphicsView(
        QGraphicsScene *scene,
        double scale = 1.05,
        QWidget *parent = nullptr
    ):
    QGraphicsView(scene, parent),
    mScale(scale > 1 ? scale : 1.05),
    mScaleInv(scale > 1 ? 1/scale : 1/1.05)
    {

    }
    virtual void wheelEvent(QWheelEvent *ev){
        if(ev->angleDelta().y() > 0) // up Wheel
            scale(mScale, mScale);
        else if(ev->angleDelta().y() < 0) //down Wheel
            scale(mScaleInv, mScaleInv);
    }
    // virtual void mouseMoveEvent(QMouseEvent *ev){
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
private:
    double mScale;
    double mScaleInv;
};


class WorkerThread : public QThread
{
    Q_OBJECT
    void run() override;
public:
    WorkerThread(
        IMS2D* p,
        double time_initial,
        double time_step,
        int steps,
        int unsigned timersleep_ms = 500
    );
    double **y;
    int getProgrees() const;
public slots:
    void emitProgress();
signals:
    void resultReady(const QString &result);
    void setProgress(int value);
private:
    IMS2D* m;
    double mTimeInitial;
    double mTimeStep;
    int mSteps;
    int mCurrentStep;
    QTimer mQTimer;
};

class SolversUpdateTimer:
public QTimer
{
    Q_OBJECT
public:
    SolversUpdateTimer(
        int unsigned timersleep_ms = 500,
        QObject *parent = nullptr
    );
    bool isFinished() const { return mFinished; }
    bool startSolvers(
        std::vector<IMS2D*> list,
        GlobalOptions::solver_options_t solver_options
    );
public slots:
    void solverupdate();
signals:
    void setProgress(int value);
private:
    int unsigned mTimerMs;
    std::vector<WorkerThread*> mThreadsList;
    bool mFinished;
    int mFinalProgess;
};

class RBViewView : public QWidget
{
    Q_OBJECT

public:
    RBViewView(
        int steps = 6000,
        QWidget *parent = 0
    );
    ~RBViewView();

    void
    addMS(
        IMS2D*
    );

private slots:
    void changeTime(
        int value
    );
public slots:
    void solve(
        GlobalOptions::solver_options_t const & solver_options
    );

private:
    QVBoxLayout mQVBoxLayout;
    QGraphicsScene mQGraphicsScene;
    PQGraphicsView mQGraphicsView;
    int mSteps;
    TimerSlider mTimerSlider;
    GlobalOptions* mGlobalOptions;
    QScrollArea* mQScrollArea; // will own mGlobalOptions
    QTabWidget mQTabWidget; // will own mQScrollArea
    std::vector<std::tuple<IMS2D*, QScrollArea*, MSOptions*>> mList;
    SolversUpdateTimer mSolversUpdateTimer;
};


class MainWindow:
public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(
        QWidget *parent = 0
    );
    ~MainWindow();

public slots:
    void addMS(
        IMS2D*
    );

private slots:
    void newFile(bool);

private:
    YAML::Node mNode;
    AddMS mAddMS;
    RBViewView mRBViewView;
    QMenuBar mQMenuBar;

    static
    YAML::Node getConfig();
};

