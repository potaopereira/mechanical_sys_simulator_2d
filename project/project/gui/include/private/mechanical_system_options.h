#pragma once

// ColorButton
#include "private/colorbutton.h"

// QGridLayout
#include <QGridLayout>

// QVBoxLayout
#include <QVBoxLayout>

// QLineEdit
#include <QLineEdit>

// QLineEdit
#include <QCheckBox>

// IMS2D
#include <msinterface/interface.hpp>

class RBOptions:
public QWidget
{
    Q_OBJECT
public:
    RBOptions(
        IMS2D* ms,
        int bodyId,
        QWidget* parent = 0
    );

    void
    set(
        int step
    );

public slots:
    void showBoundary();
    
    void
    setBoundaryColor(
        QColor color
    );

    void
    ParametersChanged(
        // 
    );

private:
    IMS2D* mIMS2D;
    int mBodyId;
    QGridLayout mQGridLayout;
    std::array<QLineEdit, 3> mPosition;
    std::array<QLineEdit, 3> mVelocity;
    std::array<QLineEdit, 3> mForce;
    std::array<QLineEdit, 3> mRelativeInput;
    std::array<QLineEdit, 3> mRelativeVelocity;
    QCheckBox mShowBoundary;
    ColorButton mColorButton;
    std::map<std::string, QLineEdit*> mParamLE;
};


class MSOptions:
public QWidget
{
    Q_OBJECT
public:
    MSOptions(
        IMS2D* ms,
        QWidget* parent = 0
    );

    void
    set(
        int step
    );

private:
    IMS2D* mIMS2D;
    QVBoxLayout mLayout;
    std::vector<RBOptions*> mV;
};
