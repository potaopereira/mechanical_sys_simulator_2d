
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
    setCentralWidget(&mMultipleMSView);
}

YAML::Node
MainWindow::getConfig(){
    try{
        // YAML::Node node = YAML::LoadFile("config_tmp.yaml");
        YAML::Node node = YAML::LoadFile("config.yaml");
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
    mMultipleMSView.addMS(ms_ptr);
}
