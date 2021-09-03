/**
 * @file timer_slider.h
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Custom slider widget (to drag from initial time instant to final time instant)
 * @version 0.1
 * @date 2021-09-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QSlider>
#include <QProgressBar>
#include <QPushButton>

class TimerSlider: public QWidget
{
    Q_OBJECT

public:

    TimerSlider(
        int steps = 1,
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

    void
    sliderMovedPre(
        int value
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