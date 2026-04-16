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



#ifndef ODRONES_GRAPHICALRNS_H
#define ODRONES_GRAPHICALRNS_H

#ifdef QT_CORE_LIB

#include <QGraphicsItem>
#include <QPainter>

#include "rns.h"
#include "constants.h"

namespace odrones {

struct graphicalSettings
{
    bool identify{false};
    bool zero{false};
    bool zeroOnly{false};
    bool allButZero{false};
};

class graphicalRNS : public QGraphicsItem
{
    struct Label
    {
        QString id;
        QPointF pos;
    };

public:
    graphicalRNS();
    graphicalRNS(const RNS &rns, const graphicalSettings &gSettings);
    ~graphicalRNS();

    void initialise();

    QRectF boundingRect() const override;

    /*! Draw the car */
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;

    bool isReady() const;

    void setup(const RNS &rns, const graphicalSettings &gSettings);

private:
    void initialiseBoundingRect(const RNS &rns);
    void setupRoadsAndLabels(const RNS &rns);
    void setupLaneAndLabel(const lane *l, uint ndx, scalar s);
    QPainterPath giveWay(const arr2 &o) const;
    QPainterPath stop(const arr2 &o) const;
    QPainterPath crossWalk(const arr2 &c, const arr2 &tg, scalar width, scalar length) const;

private:
    graphicalSettings _gs;
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
