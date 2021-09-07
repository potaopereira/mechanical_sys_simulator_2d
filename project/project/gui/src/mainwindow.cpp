
#include "private/mainwindow.h"

// std::ofstream
#include <fstream>

// std::cout
#include <iostream>

MainWindow::MainWindow(
    QWidget *parent
):
QMainWindow(parent)
,
mNode(getConfig())
,
mAddMS()
,
mMultipleMSView(mNode["MultipleMSView"])
{

    // You should then add some actions to that menu.
    //  Then you want to add the menu to the tool bar:
    QMenu * fileMenu = menuBar()->addMenu(tr("&Mechanical Systems"));

    fileMenu->addMenu(&mAddMS);
    connect(
        &mAddMS, &AddMS::addedMS,
        this, &MainWindow::addMS
    );
    // theTargetToolBar->addAction(mQMenuBar.menuAction());
    // You probably want to have some sort of icon:
    // mQMenuBar.menuAction()->setIcon(myIcon);

    setGeometry(400, 250, 542, 390);
    setCentralWidget(&mMultipleMSView);
}

YAML::Node
MainWindow::getConfig(){
    // try{
    //     YAML::Node node = YAML::LoadFile("config_tmp.yaml");
    //     // YAML::Node node = YAML::LoadFile("config.yaml");
    //     return node;
    // }
    // catch(...){
    //     std::cout << "Using default config." << std::endl;
    //     return YAML::LoadFile("config.yaml");
    // }
    try{
        YAML::Node node = YAML::LoadFile( std::string(std::getenv("HOME")) + "/config.yaml");
        std::cout << "Using config in home." << std::endl;
        return node;
    }
    catch(...){
        try{
            YAML::Node node = YAML::LoadFile( std::string(std::getenv("APPDIR")) + "/usr/bin/config.yaml");
            std::cout << "Using config in AppDir." << std::endl;
            return node;
        }
        catch(...){
            YAML::Node node = YAML::LoadFile( "config.yaml");
            std::cout << "Using config next to application." << std::endl;
            return node;
        }
    }
}

MainWindow::~MainWindow()
{
    mNode["MultipleMSView"] = mMultipleMSView.getNode();

    // std::ofstream config_file("config_tmp.yaml");
    std::ofstream config_file(std::string(std::getenv("HOME")) + "/config.yaml");
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
    mMultipleMSView.addMS(ms_ptr);
}
