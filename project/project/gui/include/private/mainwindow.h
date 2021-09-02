#pragma once

// QMainWindow
#include <QMainWindow>

// class AddMS
#include "private/addMS.h"

// MultipleMSView
#include "private/multiple_ms_view.h"

// QMenuBar
#include <QMenuBar>

// YAML::Node
#include <yaml-cpp/yaml.h>

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
    MultipleMSView mMultipleMSView;
    QMenuBar mQMenuBar;

    static
    YAML::Node getConfig();
};

