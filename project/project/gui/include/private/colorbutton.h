#pragma once

#include <QMenu>
#include <QPushButton>
#include <QColor>
#include <string>
#include <memory>
#include <QActionGroup>
#include <map>

class ColorButton:
public QPushButton
{
    Q_OBJECT
public:
    typedef std::pair<std::string, QColor> color_t;
    static std::vector<color_t> list_of_colors;
    ColorButton(
        QWidget *parent = nullptr
    );
    virtual ~ColorButton();
public slots:
    void setColor(
        QAction *action
    );
signals:
    void colorChanged(
        QColor
    );
private:
    QMenu mMenu;
    QActionGroup mQActionGroup;
    std::vector<QAction*> mSetColor;
    std::map<QAction*, color_t*> mMap;
};