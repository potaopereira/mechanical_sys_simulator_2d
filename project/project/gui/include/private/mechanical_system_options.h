/**
 * @file mechanical_system_options.h
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Widget to select mechanical system options
 * @version 0.1
 * @date 2021-08-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */

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
    void showBoundary(
        bool checked
    );
    
    void
    setBoundaryColor(
        QColor color
    );

    void
    ParametersChanged(
        // 
    );

    void changeLinearInertia(
        //
    );

    void changeAngularInertia(
        //
    );

    void showPosition(
        bool checked
    );

    void showVelocity(
        bool checked
    );

    void showForce(
        bool checked
    );

private:
    IMS2D* mIMS2D;
    int mBodyId;
    QGridLayout mQGridLayout;
    std::array<QLineEdit, 3> mInertia;
    QCheckBox mShowPosition;
    std::array<QLineEdit, 3> mPosition;
    QCheckBox mShowVelocity;
    std::array<QLineEdit, 3> mVelocity;
    QCheckBox mShowForce;
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
