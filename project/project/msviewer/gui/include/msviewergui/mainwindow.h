#pragma once

// RBView
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>

class MainWindow:
public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
private:
    QGraphicsScene mQGraphicsScene;
    QGraphicsView mQGraphicsView;
};

