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

#include "testArcLane.h"

#ifdef QT_CORE_LIB

testArcLane::testArcLane() :
    _lane(nullptr),
    _rndPoints(nullptr),
    _rndPointsSize(0)
{
    setCacheMode(DeviceCoordinateCache);

    arr2 orig = {10, 0};
    arr2 dest = {0, 10};
    arr2 centre = {20., 20.};
    dest = {dest[0] + centre[0], dest[1] + centre[1]};
    orig = {orig[0] + centre[0], orig[1] + centre[1]};

    _lane = new lane();
    _lane->set(orig, dest, centre, 3.5, 10.0, mvf::shape::counterclockwise, lane::sign::p);
    _lane->flipBackwards();
    _roads.push_back(_lane->getQPainterPath(50));

    _rndPointsSize = 20;
    _rndPoints = new arr2[_rndPointsSize];
    scalar length = _lane->getLength();
    scalar dl = length / (_rndPointsSize - 1);
    for (uint i = 0; i < _rndPointsSize; ++i)
    {
        arr2 p = {0., 0.};
        _rndPoints[i] = {0.,0.}; // safety initialisation;
        if (!_lane->getPointAfterDistance(p, _lane->getOrigin(), dl * i ))
        {
            std::cout << "[ Error ] testArcLane failed to get a point that is " << dl * i << " down the road, while length is: " << length << std::endl;
            break;
        }
        _rndPoints[i] = {p[0] + 3 * ( mvf::randomUnif01() - 0.5 ), p[1] + 3 * (mvf::randomUnif01() - 0.5)};
    }

}

testArcLane::~testArcLane()
{
    delete _lane;
    delete[] _rndPoints;
}

QRectF testArcLane::boundingRect() const
{
    // qreal adjust = 10;
    return QRectF(-3000, -3000, 6000, 6000);
}

QPainterPath testArcLane::shape() const
{
    QPainterPath path;
    path.addRect(-3000, -3000, 6000, 6000);
    return path;
}

void testArcLane::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{

    for (uint i = 0; i < _roads.size(); ++i)
        painter->drawPath(_roads[i]);

    // Bounding box:
    arr2 blc, trc;
    _lane->getBoundingBox(blc, trc);
    //  in green!
    QPen pen(Qt::green, 2);
    painter->setPen(pen);
    // Declare and get the top left corner, length and height:
    QPainterPath box;
    box.moveTo(QPointF(ct::mToPix * blc[0], - ct::mToPix * blc[1]));
    box.lineTo(QPointF(ct::mToPix * blc[0], - ct::mToPix * trc[1]));
    box.lineTo(QPointF(ct::mToPix * trc[0], - ct::mToPix * trc[1]));
    box.lineTo(QPointF(ct::mToPix * trc[0], - ct::mToPix * blc[1]));
    box.lineTo(QPointF(ct::mToPix * blc[0], - ct::mToPix * blc[1]));
    painter->drawPath(box);


    pen.setColor(Qt::cyan);
    painter->setPen(pen);
    for (uint i = 0; i < _rndPointsSize; ++i)
    {
        arr2 p;
        _lane->projectPointOntoLane(p, _rndPoints[i]);
        painter->drawLine(QPointF(ct::mToPix * _rndPoints[i][0], -ct::mToPix * _rndPoints[i][1]),
                          QPointF(ct::mToPix * p[0], -ct::mToPix * p[1]));
    }






}

#endif // QT_CORE_LIB
