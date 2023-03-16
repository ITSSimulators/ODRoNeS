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

#include "testLane.h"

#ifdef QT_CORE_LIB

testLane::testLane()
{
    setCacheMode(DeviceCoordinateCache);

    // Bezier order 3 Points:
    arr2 q0 = {12., 16.};
    arr2 q1 = {3.5, 20.};
    arr2 q2 = {22., 26.};
    arr2 q3 = {22., 4.};

    arr2 q4 = {2*q3[0] - q2[0], 2*q3[1] - q2[1]};
    arr2 q5 = {q4[0] + 12., q4[1] - 1};
    arr2 q6 = {q5[0] - 2., q5[1] + 8};

    // Create the curve:
    bezier3 b3;
    b3.set(q0, q1, q2, q3);
    std::vector<bezier3> vb3 = {bezier3(), bezier3()};
    vb3[0].set(q0, q1, q2, q3);
    vb3[1].set(q3, q4, q5, q6);
    lane lb3 = lane(vb3, 3.5, 12, lane::sign::p);

    std::vector<QPainterPath> qppv = lb3.getQPainterPaths(50);
    for (uint k = 0; k < qppv.size(); ++k)
        _roads.push_back(qppv[k]);

    QPainterPath qpp;
    qpp.moveTo(ct::mToPix * lb3.getOrigin()[0], -ct::mToPix * lb3.getOrigin()[1]);
    scalar ds = lb3.getLength() / 50.;
    for (uint i = 0; i < 50; ++i)
    {
        arr2 p;
        lb3.nGetPointAtDistance(p, i * ds);
        qpp.lineTo(ct::mToPix * p[0], -ct::mToPix * p[1]);
    }
    _roads.push_back(qpp);

    uint parts = 20;
    ds = lb3.getLength() / parts;
    for (uint i = 0; i < parts; ++i)
    {
        arr2 p;
        lb3.getPointAtDistance(p, i * ds);
        p[0] += 3 * ( mvf::randomUnif01() - 0.5);
        p[1] += 3 * ( mvf::randomUnif01() - 0.5);
        arr2 q1, q2;
        lb3.projectPointOntoLane(q1, p);
        lb3.nProjectPointHere(q2, p);
        std::cout << "[project vs project_points err]: " << mvf::distance(q1, q2) << ", p: (" << p[0] << ", " << p[1]
                  << "), q1: (" << q1[0] << ",  " << q1[1] << "), q2: (" << q2[0] << ", " << q2[1] << ")" << std::endl;
    }

}

QRectF testLane::boundingRect() const
{
    // qreal adjust = 10;
    return QRectF(-3000, -3000, 6000, 6000);
}

QPainterPath testLane::shape() const
{
    QPainterPath path;
    path.addRect(-3000, -3000, 6000, 6000);
    return path;
}

void testLane::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{

    for (uint i = 0; i < _roads.size(); ++i)
        painter->drawPath(_roads[i]);



}

#endif // QT_CORE_LIB
