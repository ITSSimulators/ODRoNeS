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

#ifndef ODRONES_GRAPHICALRNS_H
#define ODRONES_GRAPHICALRNS_H

#ifdef QT_CORE_LIB

#include <QGraphicsItem>
#include <QPainter>
#include <random>

#include "rns.h"
#include "constants.h"

namespace odrones {

class graphicalRNS : public QGraphicsItem
{
    struct Label
    {
        QString id;
        QPointF pos;
    };

public:
    graphicalRNS();
    graphicalRNS(const RNS &rns, bool identify);
    ~graphicalRNS();

    void initialise();

    QRectF boundingRect() const override;

    /*! Draw the car */
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;

    bool isReady() const;

    void setup(const RNS &rns, bool identify);

private:
    void initialiseBoundingRect(const RNS &rns);
    void setupRoadsAndLabels(const RNS &rns);
    QPainterPath giveWay(const arr2 &o) const;
    QPainterPath stop(const arr2 &o) const;
    QPainterPath crossWalk(const arr2 &c, const arr2 &tg, scalar width, scalar length) const;

private:
    bool _identifyLanes;
    uint _numberOfLanes;
    QColor *_laneColours;
    Label *_labels;
    bool *_ignore;
    std::vector<QPainterPath> _roads, _leRoads, _reRoads; ///< roads, leftEdges and rightEdges.
    uint *_subRoads; ///< the number of 'parts' (geometries) in which this lane has been divided.
    Qt::PenStyle *_penStyle;

    std::vector<lane::tSign> _tSigns;
    std::vector<conflict::staticObj> _sObjs;

    /*! bounding box, for the boundingRect()
     *   _blcX,Y means bottom left corner,
     *   _trcX,Y means top right corner. */
    QRectF _boundingBox;

    bool _ready; ///< whether it should start painting the roads or just emptyness;

    std::vector<arr2> crossingPoints; ///< DEBUG!

};

}

#endif // QT_CORE_LIB
#endif // ODRONES_GRAPHICALRNS_H
