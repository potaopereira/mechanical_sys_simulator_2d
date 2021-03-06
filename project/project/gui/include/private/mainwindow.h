/**
 * @file mainwindow.h
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Main window of gui
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

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

/**
 * @brief Main window of the gui application
 * 
 */
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

    static
    YAML::Node getConfig();
};

