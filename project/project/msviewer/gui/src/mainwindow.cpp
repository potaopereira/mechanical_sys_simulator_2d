
#include "msviewergui/mainwindow.h"

#include <msviewer/rbview.hpp>
#include <msviewer/rbviewwvectors.hpp>
#include <Eigen/Dense>
#include <cmath>

MainWindow::MainWindow(QWidget *parent) :
QMainWindow()
,
mQGraphicsScene()
,
mQGraphicsView()
{

    mQGraphicsScene.setSceneRect(-100, -100, 200, 200);
    mQGraphicsView.setMinimumWidth(500);
    mQGraphicsView.setMinimumHeight(500);
    mQGraphicsView.scale(1, -1);

    // scene takes ownership of the item
    RBView* r = new RBView();
    mQGraphicsScene.addItem(r);
    
    RBViewWVectors* q = new RBViewWVectors();
    Eigen::Matrix<double, 6, 1> p;
    p << 100, 100, +std::cos(M_PI/4), -std::sin(M_PI/4), +std::sin(M_PI/4), +std::cos(M_PI/4);
    Eigen::Matrix<double, 3, 1> v;
    v << 100, 0, 0;
    Eigen::Matrix<double, 3, 1> f;
    f << 0, 0, -100;

    q->setPoseAndVectors(
        p, v, f,
        std::vector<Eigen::Matrix<double, 2, 1>>({})
        ,
        std::vector<Eigen::Matrix<double, 2, 1>>({})
    );
    mQGraphicsScene.addItem(q);

    mQGraphicsView.setScene(&mQGraphicsScene);

    setGeometry(400, 400, 400, 400);
    setCentralWidget(&mQGraphicsView);
}

MainWindow::~MainWindow()
{
}
