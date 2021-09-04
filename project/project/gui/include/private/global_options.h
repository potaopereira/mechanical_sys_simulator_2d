/**
 * @file global_options.h
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Widget to set global options
 * @version 0.1
 * @date 2021-08-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

// IMS2D
#include <msinterface/interface.hpp>
// std::tuple, std::get
#include <tuple>
// QWidget
#include <QWidget>
// QLabel
#include <QLabel>
// QHBoxLayout
#include <QHBoxLayout>
// QVBoxLayout
#include <QVBoxLayout>
// QLineEdit
#include <QLineEdit>
// QPushButton 
#include <QPushButton>
// YAML::Node
#include <yaml-cpp/yaml.h>

class GlobalOptionFloat:
public QWidget
{
    Q_OBJECT
public:
    GlobalOptionFloat(
        std::string label,
        float min,
        float max,
        float initial,
        std::string description,
        QWidget* parent = nullptr
    );
    float getValue() const;
signals:
    void valueChanged();
private:
    QLabel mLabel;
    QLineEdit mEditor;
    QHBoxLayout mLayout;
    float mMin;
    float mMax;
};

class GlobalOptionInt:
public QWidget
{
    Q_OBJECT
public:
    GlobalOptionInt(
        std::string label,
        int min,
        int max,
        int initial,
        std::string description,
        QWidget* parent = nullptr
    );
    int getValue() const;
signals:
    void valueChanged();
private:
    QLabel mLabel;
    QLineEdit mEditor;
    QHBoxLayout mLayout;
    int mMin;
    int mMax;
};

class GlobalOptions:
public QWidget
{
    Q_OBJECT
public:
    // 
    typedef std::tuple<float, float, int unsigned> solver_options_t;
    static float getInitialTime(solver_options_t const & p) {
        return std::get<0>(p);
    }
    static float getDeltaTime(solver_options_t const & p) {
        return std::get<1>(p);
    }
    static int unsigned getSteps(solver_options_t const & p) {
        return std::get<2>(p);
    }

    GlobalOptions(
        YAML::Node const & node,
        QWidget *parent = nullptr
    );

    // virtual
    // ~GlobalOptions();

    YAML::Node getNode();

public slots:
    void requestSolver();

    void updatePlotFactors();

signals:
    void startSolver(
        solver_options_t const &
    );

    void
    plotFactorsChanged(
        IMS2D::plot_factors_t
    );


private:
    YAML::Node mNode;
    QPushButton mSolve;
    GlobalOptionFloat mInitialTime;
    GlobalOptionFloat mFinalTime;
    GlobalOptionFloat mDeltaTime;
    GlobalOptionFloat mSetLinearVelocityPlotFactor;
    GlobalOptionFloat mSetAngularVelocityPlotFactor;
    GlobalOptionFloat mSetLinearForcePlotFactor;
    GlobalOptionFloat mSetAngularForcePlotFactor;
    QVBoxLayout mLayout;

    // should be made static
    template<typename T>
    T
    getWDefault(
        YAML::Node const & node
    ){
        return node["value"].as<T, T>(node["default"].as<T>());
        // return node["value"].as<T>();
    }
};