#include "private/timer_slider.h"

#include <iostream>

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
mTInitial(QString("The start"))
,
mTEnd(QString("The end"))
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
    mQProgressBar.setMaximum(mSteps > 0 ? mSteps : 1);

    mQSlider.setMinimumWidth(50);
    mQSlider.setMaximumHeight(15);
    mQSlider.setToolTip(QString("Slide between anywhere from 'the start' to 'the end'"));
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
        this, &TimerSlider::sliderMovedPre
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
    // if(k <= mSteps){
    if(k <= mMaxCurrent){
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

void
TimerSlider::sliderMovedPre(
    int value
){
    if(value <= mMaxCurrent){
        emit sliderMoved(value);
    }
}


