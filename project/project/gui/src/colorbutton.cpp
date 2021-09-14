#include "private/colorbutton.h"

#include <QAction>

std::vector<ColorButton::color_t>
ColorButton::list_of_colors = {
    // dark red: RGBA(139, 0, 0, 1)
    std::make_pair(std::string("red"), QColor(139, 0, 0, 255))
    ,
    // dark green: RGBA(47, 79, 47, 1)
    std::make_pair(std::string("green"), QColor(47, 79, 47, 255))
    ,
    // dark blue: RGBA(0, 0, 139, 1)
    std::make_pair(std::string("blue"), QColor(0, 0, 139, 255))
    ,
    // dark orange: RGBA(255, 140, 0, 1)
    std::make_pair(std::string("orange"), QColor(255, 140, 0, 255))
    ,
    // dark brown: RGBA(92, 64, 51, 1)
    std::make_pair(std::string("brown"), QColor(92, 64, 51, 255))
    ,
    // dark purple: RGBA(135, 31, 120, 1)
    std::make_pair(std::string("purple"), QColor(135, 31, 120, 255))
};

ColorButton::ColorButton(
    QWidget *parent
):
QPushButton(QString("Color"), parent),
mMenu(QString("Color"), this),
mQActionGroup(this),
mSetColor()
{
    for(std::size_t i = 0; i < list_of_colors.size(); ++i){
        std::string color = list_of_colors[i].first;
        QAction* p = new QAction(tr(color.c_str()), this);
        mMap[p] = &list_of_colors[i];
        p->setCheckable(true);
        mMenu.addAction(p);
        mQActionGroup.addAction(p);
        mSetColor.push_back(p);
    }
    setMenu(&mMenu);

    connect(
        &mQActionGroup, &QActionGroup::triggered,
        this, &ColorButton::setColor
    );
}

ColorButton::~ColorButton(){

    const std::size_t i_max = list_of_colors.size();
    for(std::size_t i = 0; i < i_max; ++i){
        delete mSetColor[0];
        mSetColor.erase(mSetColor.begin());
    }

}

void
ColorButton::setColor(
    QAction *action
){
    color_t c = *mMap[action];
    emit colorChanged(c.second);
}