/**
 * @file mechanical_system_options.h
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Widget to select mechanical system options
 * @version 0.1
 * @date 2021
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

// QGroupBox
#include <QGroupBox>

// IMS2D
#include <msinterface/interface.hpp>

/**
 * @brief Widget to select rigid body options of a single rigid body
 * 
 */
class RBOptions:
public QGroupBox
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

    void showLinearVelocity(
        bool checked
    );

    void showAngularVelocity(
        bool checked
    );

    void showForce(
        bool checked
    );

    void showLinearForce(
        bool checked
    );

    void showAngularForce(
        bool checked
    );

    void changeRelativePoint(

    );

    void showRelativePoint(
        bool checked
    );

    void showRelativePointVelocity(
        bool checked
    );

    void showRelativePointPath(
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
    QCheckBox mShowLinearVelocity;
    QCheckBox mShowAngularVelocity;
    std::array<QLineEdit, 3> mVelocity;
    QCheckBox mShowForce;
    QCheckBox mShowLinearForce;
    QCheckBox mShowAngularForce;
    std::array<QLineEdit, 3> mForce;

    std::array<double, 2> mRelativePoint;
    std::array<QLineEdit, 3> mRelativeInput;
    QCheckBox mShowRelativePoint;
    QCheckBox mShowRelativePointPath;
    std::array<QLineEdit, 3> mRelativePointVelocity;
    QCheckBox mShowRelativePointVelocity;

    QCheckBox mShowBoundary;
    ColorButton mColorButton;
};

/**
 * @brief Widget to select parameters of the mechanical system
 * 
 */
class MSParameters:
public QGroupBox
{
    Q_OBJECT
public:
    MSParameters(
        IMS2D* ms,
        QWidget* parent = 0
    );

public slots:
    void
    ParametersChanged(
        // 
    );

private:
    IMS2D* mIMS2D;
    QGridLayout mQGridLayout;
    /**
     * @brief To change parameters of mechanical system
     * 
     */
    std::map<std::string, QLineEdit*> mParamLE;
};

/**
 * @brief Widget to holds the widgets for each rigid body of the mechanical system, plus the widget to set the parameters for the same system
 * 
 */
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
    /**
     * @brief To change parameters of mechanical system
     * 
     */
    MSParameters mMSParameters;
};
