// 
//  This file is part of the ODRoNeS (OpenDRIVE Road Network System) package.
//  
//  Copyright (c) 2023 Albert Solernou, University of Leeds.
// 
//  GTSmartActors is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
// 
//  GTSmartActors is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
// 
//  You should have received a copy of the GNU General Public License
//  along with ODRoNeS. If not, see <http://www.gnu.org/licenses/>.
// 
//  We would appreciate that if you use this software for work leading 
//  to publications you cite the package and its related publications. 
//

#ifndef TESTLANE_H
#define TESTLANE_H

#ifdef QT_CORE_LIB
#include <QGraphicsItem>
#include <QPainter>

#include "constants.h"
#include "matvec.h"
#include "lane.h"

class testLane : public QGraphicsItem
{
public:
    testLane();

    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    /*! Draw the car */
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;


private:
    std::vector<QPainterPath> _roads;


};
#endif // QT_CORE_LIB

#endif // TESTLANE_H
