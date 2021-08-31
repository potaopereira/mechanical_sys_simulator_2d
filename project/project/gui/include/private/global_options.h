
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
        QWidget *parent = nullptr
    );

public slots:
    void requestSolver();

signals:
    void startSolver(
        solver_options_t const &
    );

private:
    QPushButton mSolve;
    GlobalOptionFloat mInitialTime;
    GlobalOptionFloat mFinalTime;
    GlobalOptionFloat mDeltaTime;
    QVBoxLayout mLayout;
};