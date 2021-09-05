
#include "private/mechanical_system_options.h"

#include <QLabel>

RBOptions::RBOptions(
    IMS2D* ms,
    int bodyId,
    QWidget * parent
):
QGroupBox(ms->getName(bodyId).c_str(), parent)
,
mIMS2D(ms)
,
mBodyId(bodyId)
,
mQGridLayout()
,
mRelativePoint(std::array<double, 2>({{0,0}}))
{

    // mQGridLayout.addWidget(new QLabel( ms->getName(mBodyId).c_str()), 0, 0);

    int k = 1;
    mQGridLayout.addWidget(new QLabel("inertias"), k, 0);
    mInertia[0].setToolTip("mass kg");
    mInertia[0].setText(QString::number(ms->getLinearInertia(mBodyId), 'g', 3));
    mQGridLayout.addWidget(&mInertia[0], k, 3);
    mInertia[1].setToolTip("moment of inertia kg / mm / mm");
    mInertia[1].setText(QString::number(ms->getAngularInertia(mBodyId), 'g', 3));
    mQGridLayout.addWidget(&mInertia[1], k, 6);
    connect(
        &mInertia[0], &QLineEdit::returnPressed,
        this, &RBOptions::changeLinearInertia
    );
    connect(
        &mInertia[1], &QLineEdit::returnPressed,
        this, &RBOptions::changeAngularInertia
    );

    ++k;
    mQGridLayout.addWidget(new QLabel("position"), k, 0);
    mShowPosition.setText(QString("Show"));
    mQGridLayout.addWidget(&mShowPosition, k, 1);
    mPosition[0].setToolTip("linear position x in mm");
    mQGridLayout.addWidget(&mPosition[0], k, 3);
    mPosition[1].setToolTip("linear position y in mm");
    mQGridLayout.addWidget(&mPosition[1], k, 4);
    mPosition[2].setToolTip("angular position in degrees");
    mQGridLayout.addWidget(&mPosition[2], k, 6);
    connect(
        &mShowPosition, &QCheckBox::toggled,
        this, &RBOptions::showPosition
    );
    // by default position is shown
    mShowPosition.setChecked(true);
    showPosition(true);

    ++k;
    mQGridLayout.addWidget(new QLabel("velocity"), k, 0);
    mShowVelocity.setText(QString("All"));
    mQGridLayout.addWidget(&mShowVelocity, k, 1);
    mShowLinearVelocity.setText(QString("Linear"));
    mShowLinearVelocity.setChecked(false);
    mQGridLayout.addWidget(&mShowLinearVelocity, k, 2);
    mVelocity[0].setToolTip("linear velocity x in mm/s");
    mQGridLayout.addWidget(&mVelocity[0], k, 3);
    mVelocity[1].setToolTip("linear velocity y in mm/s");
    mQGridLayout.addWidget(&mVelocity[1], k, 4);
    mShowAngularVelocity.setText(QString("Angular"));
    mShowAngularVelocity.setChecked(false);
    mQGridLayout.addWidget(&mShowAngularVelocity, k, 5);
    mVelocity[2].setToolTip("angular velocity in degrees/s");
    mQGridLayout.addWidget(&mVelocity[2], k, 6);

    connect(
        &mShowLinearVelocity, &QCheckBox::toggled,
        this, &RBOptions::showLinearVelocity
    );
    mShowAngularVelocity.setChecked(false);
    connect(
        &mShowAngularVelocity, &QCheckBox::toggled,
        this, &RBOptions::showAngularVelocity
    );
    connect(
        &mShowVelocity, &QCheckBox::toggled,
        this, &RBOptions::showVelocity
    );
    // by default velocity is not shown
    mShowVelocity.setChecked(false);
    showVelocity(false);


    ++k;
    mQGridLayout.addWidget(new QLabel("force"), k, 0);
    mShowForce.setText(QString("All"));
    mQGridLayout.addWidget(&mShowForce, k, 1);
    mShowLinearForce.setText(QString("Linear"));
    mShowLinearForce.setChecked(false);
    mQGridLayout.addWidget(&mShowLinearForce, k, 2);
    mForce[0].setToolTip("linear force x in kg mm / s / s");
    mQGridLayout.addWidget(&mForce[0], k, 3);
    mForce[1].setToolTip("linear force y in kg mm / s / s");
    mQGridLayout.addWidget(&mForce[1], k, 4);
    mShowAngularForce.setText(QString("Angular"));
    mShowAngularForce.setChecked(false);
    mQGridLayout.addWidget(&mShowAngularForce, k, 5);
    mForce[2].setToolTip("angular force in (kg / mm / mm / s / s) times 180/pi");
    mQGridLayout.addWidget(&mForce[2], k, 6);

    connect(
        &mShowLinearForce, &QCheckBox::toggled,
        this, &RBOptions::showLinearForce
    );
    connect(
        &mShowAngularForce, &QCheckBox::toggled,
        this, &RBOptions::showAngularForce
    );
    connect(
        &mShowForce, &QCheckBox::toggled,
        this, &RBOptions::showForce
    );
    // by default force is not shown
    mShowForce.setChecked(false);
    showForce(false);

    ++k;
    mQGridLayout.addWidget(new QLabel("Point"), k, 0);
    mShowRelativePoint.setText(QString("Show"));
    mShowRelativePoint.setToolTip("Show point on rigid body");
    mShowRelativePointPath.setText(QString("Path"));
    mShowRelativePointPath.setToolTip("Show path of point on rigid body");
    mQGridLayout.addWidget(&mShowRelativePoint, k, 1);
    mQGridLayout.addWidget(&mShowRelativePointPath, k, 2);
    mQGridLayout.addWidget(&mRelativeInput[0], k, 3);
    mQGridLayout.addWidget(&mRelativeInput[1], k, 4);
    // mQGridLayout.addWidget(mRelative[2], k, 5);
    for(int i = 0; i < 2; ++i)
        connect(
            &mRelativeInput[i], &QLineEdit::returnPressed,
            this, &RBOptions::changeRelativePoint
        );
    connect(
        &mShowRelativePoint, &QCheckBox::toggled,
        this, &RBOptions::showRelativePoint
    );
    connect(
        &mShowRelativePointPath, &QCheckBox::toggled,
        this, &RBOptions::showRelativePointPath
    );
    // by default, relative point is not shown
    mShowRelativePoint.setChecked(false);
    showRelativePoint(false);

    ++k;
    mQGridLayout.addWidget(new QLabel("Point velocity"), k, 0);
    mShowRelativePointVelocity.setText(QString("Show"));
    mShowRelativePointVelocity.setToolTip("Show velocity of point on rigid body");
    mQGridLayout.addWidget(&mShowRelativePointVelocity, k, 1);
    mQGridLayout.addWidget(&mRelativePointVelocity[0], k, 3);
    mQGridLayout.addWidget(&mRelativePointVelocity[1], k, 4);
    // mQGridLayout.addWidget(mRelative[2], k, 5);
    connect(
        &mShowRelativePointVelocity, &QCheckBox::toggled,
        this, &RBOptions::showRelativePointVelocity
    );
    // by default, relative point velocity is not shown
    mShowRelativePointVelocity.setChecked(false);
    showRelativePointVelocity(false);

    ++k;
    mQGridLayout.addWidget(new QLabel("Boundary"), k, 0);
    mShowBoundary.setText(QString("Show"));
    mQGridLayout.addWidget(&mShowBoundary, k, 1);
    mQGridLayout.addWidget(&mColorButton, k, 2);
    connect(
        &mColorButton, &ColorButton::colorChanged,
        this, &RBOptions::setBoundaryColor
    );
    connect(
        &mShowBoundary, &QCheckBox::toggled,
        this, &RBOptions::showBoundary
    );
    // by default boundary is shown
    mShowBoundary.setChecked(true);
    showBoundary(true);

    setLayout(&mQGridLayout);
}

void
RBOptions::showBoundary(
    bool checked
){
    mIMS2D->showBoundary(mBodyId, checked);
    // mIMS2D->showBoundary(mBodyId, mShowBoundary.isChecked());
}

void
RBOptions::setBoundaryColor(
    QColor color
){
    mIMS2D->setBoundaryColor(mBodyId, color);
}

void
RBOptions::set(
    int step
){
    std::array<double, 3> p = mIMS2D->getPosition(step, mBodyId);
    std::array<double, 3> v = mIMS2D->getVelocity(step, mBodyId);
    std::array<double, 3> f = mIMS2D->getForce(step, mBodyId);
    std::array<double, 2> q = mIMS2D->getVelocityOfPoint(step, mBodyId, mRelativePoint);

    for(int j = 0; j < 3; ++j){
        mPosition[j].setText(QString::number(p[j], 'g', 3));
        mVelocity[j].setText(QString::number(v[j], 'g', 3));
        mForce[j].setText(QString::number(f[j], 'g', 3));
    }

    mRelativePointVelocity[0].setText(QString::number(q[0], 'g', 3));
    mRelativePointVelocity[1].setText(QString::number(q[1], 'g', 3));
}

void
RBOptions::changeLinearInertia(){

    double mass = mInertia[0].text().toFloat();
    if(mass>0)
        mIMS2D->setLinearInertia(mass, mBodyId);

}

void
RBOptions::changeAngularInertia(){

    double moment_of_inertia = mInertia[1].text().toFloat();
    if(moment_of_inertia>0)
        mIMS2D->setAngularInertia(moment_of_inertia, mBodyId);

}

void
RBOptions::showPosition(
    bool checked
){
    mIMS2D->showPosition(mBodyId, checked);
    // mIMS2D->showPosition(mBodyId, mShowPosition.isChecked());
}

void
RBOptions::showVelocity(
    bool checked
){
    mIMS2D->showVelocity(mBodyId, checked);
    // mIMS2D->showVelocity(mBodyId, mShowVelocity.isChecked());
}

void
RBOptions::showLinearVelocity(
    bool checked
){
    mIMS2D->showLinearVelocity(mBodyId, checked);
}

void
RBOptions::showAngularVelocity(
    bool checked
){
    mIMS2D->showAngularVelocity(mBodyId, checked);
}

void
RBOptions::showForce(
    bool checked
){
    mIMS2D->showForce(mBodyId, checked);
    // mIMS2D->showForce(mBodyId, mShowForce.isChecked());
}

void
RBOptions::showLinearForce(
    bool checked
){
    mIMS2D->showLinearForce(mBodyId, checked);
}

void
RBOptions::showAngularForce(
    bool checked
){
    mIMS2D->showAngularForce(mBodyId, checked);
}

void
RBOptions::changeRelativePoint(
){
    mRelativePoint = std::array<double, 2>(
        {{
            mRelativeInput[0].text().toFloat(),
            mRelativeInput[1].text().toFloat()
        }}
    );
    if(mShowRelativePoint.isChecked())
        showRelativePoint(true);
    if(mShowRelativePointVelocity.isChecked())
        showRelativePointVelocity(true);
    if(mShowRelativePointPath.isChecked())
        showRelativePointPath(true);
}

void
RBOptions::showRelativePoint(
    bool checked
){
    mIMS2D->showRelativePoint(checked, mBodyId, mRelativePoint);
}

void
RBOptions::showRelativePointVelocity(
    bool checked
){
    mIMS2D->showRelativePointVelocity(checked, mBodyId, mRelativePoint);
}

void
RBOptions::showRelativePointPath(
    bool checked
){
    mIMS2D->showRelativePointPath(
        checked, mBodyId, mRelativePoint,
        300 // number of points forming path
    );
}


MSParameters::MSParameters(
    IMS2D* ms,
    QWidget * parent
):
QGroupBox("Parameters", parent)
,
mIMS2D(ms)
,
mQGridLayout()
{
    /*
    parameters of mechanical system
    */
    IMRB2DPARAM::map_t m = mIMS2D->getFloatParameters();
    int k = 0;
    for(IMRB2DPARAM::map_t::iterator it = m.begin(); it!=m.end(); ++it){
        IMRB2DPARAM::floatparam_t const floatparam = it->second;
        QLabel* ql = new QLabel( QString::fromStdString(IMRB2DPARAM::getName(floatparam)) );
        ql->setToolTip(QString::fromStdString(IMRB2DPARAM::getDescription(floatparam)));
        mQGridLayout.addWidget(ql, k, 1);
        QLineEdit* qle = new QLineEdit();
        mParamLE[it->first] = qle;
        qle->setText(QString::number(IMRB2DPARAM::getValue(floatparam), 'g', 3));
        
        char msg[40];
        sprintf(msg, "Between %+.3f and %+.3f", IMRB2DPARAM::getMin(floatparam), IMRB2DPARAM::getMax(floatparam));
        qle->setToolTip(QString(msg));
        mQGridLayout.addWidget(qle, k, 2);

        connect(
            qle, &QLineEdit::returnPressed,
            this, &MSParameters::ParametersChanged
        );

        /*increment index use to place widgets in layout*/
        ++k;
    }
    setLayout(&mQGridLayout);
}


void
MSParameters::ParametersChanged(

){

    IMRB2DPARAM::map_t m = mIMS2D->getFloatParameters();
    for(IMRB2DPARAM::map_t::iterator it = m.begin(); it!=m.end(); ++it){
        std::string const floatparamname = it->first;
        mIMS2D->setFloatParameter(
            floatparamname,
            mParamLE[floatparamname]->text().toFloat()
        );
    }

}

MSOptions::MSOptions(
    IMS2D* ms,
    QWidget * parent
):
QWidget(parent)
,
mIMS2D(ms)
,
mLayout()
,
mMSParameters(mIMS2D)
{

    for(int i = 0; i < mIMS2D->dimRigidBodies(); ++i){

        RBOptions* p = new RBOptions(ms, i);
        mV.push_back(p);
        mLayout.addWidget(p);

    }

    mLayout.addWidget(&mMSParameters);
    setLayout(&mLayout);
}

void
MSOptions::set(
    int step
){
    for(int i = 0; i < mIMS2D->dimRigidBodies(); ++i){
        mV[i]->set(step);
    }
}