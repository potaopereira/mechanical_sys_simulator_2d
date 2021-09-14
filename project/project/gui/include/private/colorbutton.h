/**
 * @file colorbutton.h
 * @author Pedro Pereira (pedro.m.otao.pereira@gmail.com)
 * @brief Menu to select color from a list of available colors
 * @version 0.1
 * @date 2021
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once

#include <QMenu>
#include <QPushButton>
#include <QColor>
#include <string>
#include <memory>
#include <QActionGroup>
#include <map>

/**
 * @brief Menu to hold available colors
 * 
 */
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