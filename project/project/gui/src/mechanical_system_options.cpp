
#include "private/mechanical_system_options.h"

#include <QLabel>

RBOptions::RBOptions(
    IMS2D* ms,
    int bodyId,
    QWidget * parent
):
QWidget(parent)
,
mIMS2D(ms)
,
mBodyId(bodyId)
,
mQGridLayout()
// ,
// mPosition()
{
    connect(
        &mShowBoundary, &QCheckBox::pressed,
        this, &RBOptions::showBoundary
    );

    mQGridLayout.addWidget(new QLabel((std::string("i")+std::to_string(bodyId)).c_str()), 0, 0);

    int k = 1;
    mQGridLayout.addWidget(new QLabel("position"), k, 1);
    mQGridLayout.addWidget(&mPosition[0], k, 2);
    mQGridLayout.addWidget(&mPosition[1], k, 3);
    mQGridLayout.addWidget(&mPosition[2], k, 4);

    ++k;
    mQGridLayout.addWidget(new QLabel("velocity"), k, 1);
    mQGridLayout.addWidget(&mVelocity[0], k, 2);
    mQGridLayout.addWidget(&mVelocity[1], k, 3);
    mQGridLayout.addWidget(&mVelocity[2], k, 4);

    ++k;
    mQGridLayout.addWidget(new QLabel("force"), k, 1);
    mQGridLayout.addWidget(&mForce[0], k, 2);
    mQGridLayout.addWidget(&mForce[1], k, 3);
    mQGridLayout.addWidget(&mForce[2], k, 4);

    ++k;
    mQGridLayout.addWidget(new QLabel("point"), k, 1);
    mQGridLayout.addWidget(&mRelativeInput[0], k, 2);
    mQGridLayout.addWidget(&mRelativeInput[1], k, 3);
    // mQGridLayout.addWidget(mRelative[2], k, 4);

    ++k;
    mQGridLayout.addWidget(new QLabel("point velocity"), k, 1);
    mQGridLayout.addWidget(&mRelativeVelocity[0], k, 2);
    mQGridLayout.addWidget(&mRelativeVelocity[1], k, 3);
    // mQGridLayout.addWidget(mRelative[2], k, 4);

    ++k;
    mQGridLayout.addWidget(new QLabel("show boundary"), k, 1);
    mQGridLayout.addWidget(&mShowBoundary, k, 2);
    // by default boundary is shown
    mShowBoundary.setDown(false);
    mQGridLayout.addWidget(&mColorButton, k, 3);
    connect(
        &mColorButton, &ColorButton::colorChanged,
        this, &RBOptions::setBoundaryColor
    );

    IMRB2DPARAM::map_t m = mIMS2D->getFloatParameters();
    for(IMRB2DPARAM::map_t::iterator it = m.begin(); it!=m.end(); ++it){
        ++k;
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
            this, &RBOptions::ParametersChanged
        );

    }

    setLayout(&mQGridLayout);
}

void
RBOptions::showBoundary(

){
    mIMS2D->showBoundary(mBodyId, mShowBoundary.isChecked());
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
    std::array<double, 2> q = mIMS2D->getVelocityOfPoint(step, mBodyId, std::array<double,2>({{0,0}}));

    for(int j = 0; j < 3; ++j){
        mPosition[j].setText(QString::number(p[j], 'g', 3));
        mVelocity[j].setText(QString::number(v[j], 'g', 3));
        mForce[j].setText(QString::number(f[j], 'g', 3));
    }

    mRelativeVelocity[0].setText(QString::number(q[0], 'g', 3));
    mRelativeVelocity[1].setText(QString::number(q[1], 'g', 3));
}

void
RBOptions::ParametersChanged(

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
// ,
// mPosition()
{

    for(int i = 0; i < mIMS2D->dimRigidBodies(); ++i){

        RBOptions* p = new RBOptions(ms, i);
        mV.push_back(p);
        mLayout.addWidget(p);

    }
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
