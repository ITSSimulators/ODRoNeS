//
//   This file is part of ODRoNeS (OpenDRIVE Road Network System).
//
//   Copyright (c) 2019-2026 Albert Solernou, University of Leeds.
//
//   The ODRoNeS package is free software; you can redistribute it and/or
//   modify it under the terms of the GNU Lesser General Public
//   License as published by the Free Software Foundation; either
//   version 3 of the License, or (at your option) any later version.
//
//   The ODRoNeS package is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//   Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public
//   License along with the ODRoNeS package; if not, see
//   <https://www.gnu.org/licenses/>.
//



#ifndef ODRONES_TESTBEZIER_H
#define ODRONES_TESTBEZIER_H

#ifdef QT_CORE_LIB
#include <QGraphicsItem>
#include <QPainter>

#include "constants.h"
#include "bezier2.h"
#include "bezier3.h"
#include "matvec.h"

#include <random>

class testBezier : public QGraphicsItem
{
public:
    testBezier();

    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    /*! Draw the car */
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;

private:

    odrones::scalar randomUnif01();
    std::minstd_rand _randomGenerator;
    std::uniform_real_distribution<> _unifDist01;
};

#endif // QT_CORE_LIB

#endif // ODRONES_TESTBEZIER_H
