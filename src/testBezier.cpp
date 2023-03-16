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

#include "testBezier.h"

#ifdef QT_CORE_LIB

testBezier::testBezier()
{
    setCacheMode(DeviceCoordinateCache);
}

QRectF testBezier::boundingRect() const
{
    // qreal adjust = 10;
    return QRectF(-3000, -3000, 6000, 6000);
}

QPainterPath testBezier::shape() const
{
    QPainterPath path;
    path.addRect(-3000, -3000, 6000, 6000);
    return path;
}

void testBezier::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{


    // ORDER 2:
    // Points:
    arr2 p0 = {7., 25.};
    arr2 p1 = {2., 11.};
    arr2 p2 = {25., 6.};

    /*
    // Draw the three points:
    QPoint Q[3];
    Q[0] = QPoint(p0[0], p0[1]);
    Q[1] = QPoint(p1[0], p1[1]);
    Q[2] = QPoint(p2[0], p2[1]);
    for (size_t i = 0; i < 3; ++i)
        Q[i] *= ct::mToPixD;

    painter->drawPoints(Q, 3);
    */

    // Create the curve:
    bezier2 lb2;
    lb2.set(p0, p1, p2);
    // And draw it using one hundred points:
    painter->drawPath(lb2.getQPainterPath(100));


    // ORDER 3:
    // Points:
    arr2 q0 = {12., 16.};
    arr2 q1 = {3.5, 20.};
    arr2 q2 = {22., 26.};
    arr2 q3 = {22., 4.};

    // Create the curve:
    bezier3 lb3;
    lb3.set(q0, q1, q2, q3);
    // And draw it using 100 points:
    painter->drawPath(lb3.getQPainterPath(100));


    // Length of the curves:
    scalar lb2Length = lb2.getLength();
    std::cout << "length for lb2: " << lb2Length << std::endl;
    scalar lb3Length = lb3.getLength();
    std::cout << "length for lb3: " << lb3Length << std::endl;


    // Split the curves at "split" and draw the two bits:
    scalar split = 0.3;
    bezier2 bs2 = lb2.getStartingPart(split);
    bezier2 be2 = lb2.getEndingPart(split);
    bezier3 bs3 = lb3.getStartingPart(split);
    bezier3 be3 = lb3.getEndingPart(split);

    QPen pen(Qt::blue, 3); // , Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    painter->setPen(pen);
    painter->drawPath(bs2.getQPainterPath(100));
    painter->drawPath(bs3.getQPainterPath(100));
    pen.setColor(Qt::red);
    painter->setPen(pen);
    painter->drawPath(be2.getQPainterPath(100));
    painter->drawPath(be3.getQPainterPath(100));



    // Check whether some of the values are within the [0, 1] range:
    if (!lb2.isTinRange(0.3))
        std::cerr << "[2] wrong: 0.3 should be in range " << std::endl;
    if (!lb2.isTinRange(0.7))
        std::cerr << "[2] wrong: 0.7 should be in range " << std::endl;
    if (lb2.isTinRange(-1))
        std::cerr << "[2] wrong: -1 should not be in range " << std::endl;
    if (lb2.isTinRange(1.1))
        std::cerr << "[2] wrong: 1.1 should not be in range " << std::endl;


    // RECOVER t from (x, y)
    // Split the curve in parts, and check that we can recover t from (x, y) repeatedly.
    // Order 2:
    uint parts = 1000;
    scalar dt = 1. / parts;
    for (uint i = 0; i <= parts; ++i)
    {
        scalar t = i * dt;
        arr2 xy = lb2.curvexy(t);
        scalar tTest = 0;
        if (!lb2.getTGivenXY(tTest, xy))
        {
            std::cout << "[2] no t found for xy: " << xy[0] << ", " << xy[1] << " really corresponding to t: " << t << std::endl;
            continue;
        }
        if (!mvf::areSameValues(t, tTest))
        {
            std::cout << "[2] error! we found " << tTest << " instead of " << t << std::endl;
            continue;
        }
    }


    // Order 3:
    for (uint i = 0; i <= parts; ++i)
    {
        scalar t = i * dt;
        arr2 xy = lb3.curvexy(t);
        scalar tTest = 0;
        if (!lb3.getTGivenXY(tTest, xy))
        {
            std::cout << "[3] no t found for xy: " << xy[0] << ", " << xy[1] << " really corresponding to t: " << t << std::endl;
            continue;
        }
        if (!mvf::areSameValues(t, tTest))
        {
            std::cout << "[3] error! we found " << tTest << " instead of " << t << std::endl;
            continue;
        }
        // std::cout << "[3] Ace: we found that " << t << " matches " << tTest << std::endl;
    }



    // RECOVER t given a certain distance down the line,
    // Orders 2 and 3:
    for (uint i = 0; i <= parts; ++i)
    {
        scalar ti;
        scalar d = i*lb3.getLength()/parts;

        if (!lb3.getTgivenD(ti, d))
        {
            std::cerr << "we couldn't get t: " << ti << " for d: " << d << std::endl;
        }

        scalar db = i*lb2.getLength()/parts;
        if (!lb2.getTgivenD(ti, db))
        {
            std::cerr << "we couldn't get t: " << ti << " for d: " << d << std::endl;
        }

    }


    // PLACE a point that is some distance down from a previous one:
    // Order 3:
    scalar Ds = lb3.getLength()/parts;
    scalar to = 0;
    arr2 o = lb3.curvexy(to);
    for (uint i = 1; i <= parts; ++i)
    {
        arr2 p;
        lb3.getPointAfterDistance(p, o, Ds);
        scalar ti;
        lb3.getTGivenXY(ti, p);
        scalar si = lb3.distanceBetween(to, ti);
        if (!mvf::areSameValues(si, Ds))
        {
            std::cerr << "wrong distance: " << si << " does not match Ds: " << Ds
                      << ", abs err: " << std::abs(Ds-si) <<  std::endl ;
        }
        // else
            // std::cout << "great work: " << si << " does matches Ds: " << Ds << std::endl;
        to = ti;
        o = {p[0], p[1]};
    }


    // Order 2:
    Ds = lb2.getLength()/parts;
    to = 0;
    o = lb2.curvexy(to);
    for (uint i = 1; i <= parts; ++i)
    {
        arr2 p;
        lb2.getPointAfterDistance(p, o, Ds);
        scalar ti;
        lb2.getTGivenXY(ti, p);
        scalar si = lb2.distanceBetween(to, ti);
        if (!mvf::areSameValues(si, Ds))
        {
            std::cerr << "wrong distance: " << si << " does not match Ds: " << Ds
                      << ", abs err: " << std::abs(Ds-si) <<  std::endl ;
        }
        // else
            // std::cout << "great work: " << si << " does matches Ds: " << Ds << std::endl;
        to = ti;
        o = {p[0], p[1]};
    }


    // CHECK that the distance is the same for the split curve and for the calculated one:
    for (uint i = 0; i <= parts; ++i)
    {
        scalar t = i * dt;
        scalar s_o = lb2.calcLength(t);
        bezier2 startB2 = lb2.getStartingPart(t);
        if (!mvf::areSameValues(s_o, startB2.getLength()))
        {
            std::cerr << "[2] wrong partial distance at t: " << t << ": got " << s_o << " instead of " << startB2.getLength() << std::endl;
        }

        s_o = lb3.calcLength(t);
        bezier3 startB3 = lb3.getStartingPart(t);
        if (!mvf::areSameValues(s_o, startB3.getLength()))
        {
            std::cerr << "[3] wrong partial distance at t: " << t << ": got " << s_o << " instead of " << startB3.getLength() << std::endl;
        }

    }


    // Get the BOUNDING BOX:
    //  in green!
    pen.setColor(Qt::green);
    pen.setWidth(2);
    painter->setPen(pen);
    // Declare and get the top left corner, length and height:
    arr2 blc, trc;
    lb2.getBoundingBox(blc, trc);
    arr2 tlc = {blc[0], trc[1]};
    scalar height = trc[1] - blc[1];
    scalar length = trc[0] - blc[0];
    QPainterPath bz2Box;
    bz2Box.moveTo(QPointF(ct::mToPix * tlc[0], - ct::mToPix * tlc[1]));
    bz2Box.lineTo(QPointF(ct::mToPix * (tlc[0] + length), - ct::mToPix * tlc[1]));
    bz2Box.lineTo(QPointF(ct::mToPix * (tlc[0] + length), - ct::mToPix * (tlc[1] - height)));
    bz2Box.lineTo(QPointF(ct::mToPix * tlc[0], - ct::mToPix * (tlc[1] - height)));
    bz2Box.lineTo(QPointF(ct::mToPix * tlc[0], - ct::mToPix * tlc[1]));
    painter->drawPath(bz2Box);
    std::cout << "[2] bounding box: " << tlc[0] << ", " << tlc[1] << ", l: " << length << " and h: " << height << std::endl;

    // Now O3:
    // Declare and get the top left corner, length and height:
    lb3.getBoundingBox(blc, trc);
    tlc = {blc[0], trc[1]};
    height = trc[1] - blc[1];
    length = trc[0] - blc[0];
    QPainterPath bz3Box;
    std::cout << "[3] bounding box: " << tlc[0] << ", " << tlc[1] << ", l: " << length << " and h: " << height << std::endl;
    bz3Box.moveTo(QPointF(ct::mToPix * tlc[0], - ct::mToPix * tlc[1]));
    bz3Box.lineTo(QPointF(ct::mToPix * (tlc[0] + length), - ct::mToPix * tlc[1]));
    bz3Box.lineTo(QPointF(ct::mToPix * (tlc[0] + length), - ct::mToPix * (tlc[1] - height)));
    bz3Box.lineTo(QPointF(ct::mToPix * tlc[0], - ct::mToPix * (tlc[1] - height)));
    bz3Box.lineTo(QPointF(ct::mToPix * tlc[0], - ct::mToPix * tlc[1]));
    painter->drawPath(bz3Box);



    // Test the INTERSECTIONS:
    // Order 2:
    o = {ct::oneThird * (p0[0] + p1[0] + p2[0]), ct::oneThird * ( p0[1] + p1[1] + p2[1]) };
    arr2 t = {-1, -1};
    mvf::resize(t, 1);

    // Draw in black the origin and tangent we're using for the intersection:
    pen.setColor(Qt::black);
    painter->setPen(pen);
    painter->drawLine(QPointF(ct::mToPix * o[0], -ct::mToPix * o[1]), QPointF(ct::mToPix * (o[0] + t[0]), -ct::mToPix * (o[1] + t[1])));


    arr2 p;
    if (lb2.getIntersectionPointFromOT(p, o, t))
    {
        std::cout << "[2] intersection found at: " << p[0] << ":" << p[1] << std::endl;
        pen.setColor(Qt::gray);
        painter->setPen(pen);
        painter->drawLine(QPointF(ct::mToPix * (o[0] + t[0]), -ct::mToPix * (o[1] + t[1])), QPointF(ct::mToPix * p[0], -ct::mToPix * p[1]));
    }
    else
    {
        std::cout << "[2] no intersection found" << std::endl;
    }


    // Order 3:
    o = {0.25 * (q0[0] + q1[0] + q2[0] + q3[0]), 0.25 * ( q0[1] + q1[1] + q2[1] + q3[1]) };
    t = {1, 1};
    mvf::resize(t, 1);

    // Draw in black the origin and tangent we're using for the intersection:
    pen.setColor(Qt::black);
    painter->setPen(pen);
    painter->drawLine(QPointF(ct::mToPix * o[0], -ct::mToPix * o[1]), QPointF(ct::mToPix * (o[0] + t[0]), -ct::mToPix * (o[1] + t[1])));

    if (lb3.getIntersectionPointFromOT(p, o, t))
    {
        std::cout << "[3] intersection found at: " << p[0] << ":" << p[1] << std::endl;
        pen.setColor(Qt::gray);
        painter->setPen(pen);
        painter->drawLine(QPointF(ct::mToPix * (o[0] + t[0]), -ct::mToPix * (o[1] + t[1])), QPointF(ct::mToPix * p[0], -ct::mToPix * p[1]));
    }
    else
    {
        std::cout << "[3] no intersection found" << std::endl;
    }

    // Test the PROJECTIONS:
    pen.setColor(Qt::magenta);
    parts = 20;
    for (uint i = 0; i < parts; ++i)
    {
        pen.setColor(Qt::cyan);
        painter->setPen(pen);
        arr2 xy = lb2.curvexy(i * 1.0 / parts);
        xy[0] += 3 * ( mvf::randomUnif01() - 0.5);
        xy[1] += 3 * ( mvf::randomUnif01() - 0.5);
        arr2 p, nrm, tg;
        scalar t = lb2.projectPointHereT(p, xy);

        tg = lb2.curvePxy(t);
        mvf::normalise(tg);
        mvf::tangent(nrm, xy, p);
        if (std::fabs(nrm[0]*tg[0] + nrm[1]*tg[1]) > 1e-2)
        {
            pen.setColor(Qt::magenta);
            painter->setPen(pen);
            std::cout << "too much: " << o[0] << ", " << o[1] << " went to " << p[0] << ", " << p[1] << " at " << t
                      << " but tg: " << tg[0] << ", " << tg[1] << " and nrm: " << nrm[0] << ", " << nrm[1]
                      << " do not add up: " << nrm[0]*tg[0] + nrm[1]*tg[1] << " d: " << mvf::distance(p, o) << std::endl;

        }
        painter->drawLine(QPointF(ct::mToPix * xy[0], -ct::mToPix * xy[1]), QPointF(ct::mToPix * p[0], -ct::mToPix * p[1]));
    }

    for (uint i = 0; i < parts; ++i)
    {
        pen.setColor(Qt::cyan);
        painter->setPen(pen);
        arr2 xy = lb3.curvexy(i * 1.0 / parts);
        xy[0] += 3 * ( mvf::randomUnif01() - 0.5);
        xy[1] += 3 * ( mvf::randomUnif01() - 0.5);
        arr2 p, nrm, tg;
        scalar t = lb3.projectPointHereT(p, xy);

        tg = lb3.curvePxy(t);
        mvf::normalise(tg);
        mvf::tangent(nrm, xy, p);
        if (std::fabs(nrm[0]*tg[0] + nrm[1]*tg[1]) > 1e-2)
        {
            pen.setColor(Qt::magenta);
            painter->setPen(pen);
            std::cout << "too much: " << o[0] << ", " << o[1] << " went to " << p[0] << ", " << p[1] << " at " << t
                      << " but tg: " << tg[0] << ", " << tg[1] << " and nrm: " << nrm[0] << ", " << nrm[1]
                      << " do not add up: " << nrm[0]*tg[0] + nrm[1]*tg[1] << " d: " << mvf::distance(p, o) << std::endl;

        }
        painter->drawLine(QPointF(ct::mToPix * xy[0], -ct::mToPix * xy[1]), QPointF(ct::mToPix * p[0], -ct::mToPix * p[1]));
    }






}

#endif // QT_CORE_LIB
