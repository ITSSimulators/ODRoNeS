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

#ifdef QT_CORE_LIB

#include "graphicalrns.h"

graphicalRNS::graphicalRNS()
{
    initialise();
}

graphicalRNS::graphicalRNS(const RNS &rns, bool identify)
{
    initialise();
    setup(rns, identify);
}

void graphicalRNS::initialise()
{
    _numberOfLanes = 0;
    _boundingBox = QRectF(-3000,-3000,6000,6000);
    _ready = false;
    setCacheMode(DeviceCoordinateCache);
}

graphicalRNS::~graphicalRNS()
{
    if (_numberOfLanes > 0)
    {
        delete[] _laneColours;
        delete[] _subRoads;
        delete[] _penStyle;
        delete[] _ignore;
        if (_identifyLanes) delete[] _labels;
    }

}

void graphicalRNS::setup(const RNS &rns, bool identify)
{
    _identifyLanes = identify;

    _numberOfLanes = rns.lanesSize();
    if (_numberOfLanes == 0)
    {
        std::cerr << "[ gRNS ] there's nothing to plot, lrn->getNumberOfLanes() returned zero. " << std::endl;
        return;
    }
    _laneColours = new QColor[_numberOfLanes];

    initialiseBoundingRect(rns);
    setupRoadsAndLabels(rns);

    std::minstd_rand rg(1659);
    std::uniform_int_distribution<int> u0255(0, 255);
    for (uint i = 0; i < _numberOfLanes; ++i)
        _laneColours[i] = QColor(u0255(rg), u0255(rg), u0255(rg));

    _ready = true;
}

QRectF graphicalRNS::boundingRect() const
{
    return _boundingBox;
}

void graphicalRNS::initialiseBoundingRect(const RNS &rns)
{
    scalar margin = 10;
    scalar blcX, blcY, trcX, trcY;
    rns.getDimensions(blcX, blcY, trcX, trcY);
    scalar w = ct::mToPix * ( (trcX - blcX) + 2 * margin );
    scalar h = ct::mToPix * ( (trcY - blcY) + 2 * margin );
    _boundingBox = QRectF(ct::mToPix * (blcX - margin), - ct::mToPix * (trcY + margin), w, h);
}

bool graphicalRNS::isReady() const
{
    return _ready;
}

void graphicalRNS::setupRoadsAndLabels(const RNS &rns)
{
    if (_identifyLanes)
        _labels = new Label[_numberOfLanes];
    _subRoads = new uint[_numberOfLanes];
    _penStyle = new Qt::PenStyle[_numberOfLanes];
    _ignore = new bool[_numberOfLanes];
    std::fill_n(_ignore, _numberOfLanes, false);

    uint laneNumber = 0;
    for (unsigned int i=0; i<rns.sectionsSize(); ++i)
    {
        for (unsigned int j=0; j<rns.sections(i).size(); ++j)
        {
            lane *l = rns.sections(i)[j];
            if ( (l->getKind() == lane::kind::tarmac) || (l->getKind() == lane::kind::roundabout) )
                _penStyle[laneNumber] = Qt::DashLine;
            else if ( ( l->getKind() == lane::kind::pavement) || (l->getKind() == lane::kind::crosswalk) )
                _penStyle[laneNumber] = Qt::DotLine;
            else
            {
                _ignore[laneNumber] = true;
                _penStyle[laneNumber] = Qt::SolidLine;
            }


            if (_identifyLanes) // put an ID to every lane:
            {
                arr2 o = l->getOrigin();
                arr2 halfway;
                l->getPointAfterDistance(halfway, o, 0.5*l->getLength() + l->getID());
                _labels[laneNumber].pos = {ct::mToPix * halfway[0], - ct::mToPix * halfway[1]};
                std::string laneID = l->getCSUID();
                _labels[laneNumber].id = laneID.c_str();
            }

            if (l->getGeometrySize() < 2)
            {
                _roads.push_back(l->getQPainterPath(50));
                _subRoads[laneNumber] = 1;
            }
            else
            {
                std::vector<QPainterPath> qpp = l->getQPainterPaths(50);
                _subRoads[laneNumber] = static_cast<uint>(qpp.size());
                for (unsigned int k=0; k < qpp.size(); ++k)
                    _roads.push_back(qpp[k]);
            }
            _leRoads.push_back(l->getEdgeQPainterPath(50, -1));
            _reRoads.push_back(l->getEdgeQPainterPath(50, 1));

            laneNumber += 1;
        }
    }

    _tSigns = rns.tSigns();
    // _sObjs = glrn->getStaticObjs();

    crossingPoints = rns.crossingPoints;
}

void graphicalRNS::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{

    if (!isReady()) return;

    QPen pen(Qt::blue, 2, Qt::DashLine, Qt::RoundCap, Qt::RoundJoin);
    pen.setColor(Qt::gray);
    painter->setPen(pen);


    QFont font = painter->font();
    font.setPointSizeF(0.5 * font.pointSizeF());
    painter->setFont(font);

    uint srNdx = 0;
    for (uint i = 0; i < _numberOfLanes; ++i)
    {
        pen.setStyle(_penStyle[i]); // dashLine for roads, dashDotLines for sidewalks.
        if (_identifyLanes)
        {
            pen.setColor(_laneColours[i]);
            painter->setPen(pen);
            if (!_ignore[i])
                painter->drawText(_labels[i].pos, _labels[i].id);
        }
        for (uint j = 0; j < _subRoads[i]; ++j)
        {
            if (!_ignore[i])
                painter->drawPath(_roads[srNdx]);
            srNdx += 1;
        }
        if (!_ignore[i])
        {
            pen.setStyle(Qt::DotLine);
            pen.setColor(Qt::red);
            painter->setPen(pen);
            painter->drawPath(_leRoads[i]);
            pen.setColor(Qt::black);
            painter->setPen(pen);
            painter->drawPath(_reRoads[i]);
        }
    }

    for (uint i = 0; i < _tSigns.size(); ++i)
    {
        if (_tSigns[i].info == lane::tSignInfo::giveWay)
        {
            if (_tSigns[i].assigned) pen.setColor(Qt::red);
            else pen.setColor(Qt::black);
            painter->setPen(pen);
            painter->drawPath(giveWay(_tSigns[i].pos));
        }
        else if (_tSigns[i].info == lane::tSignInfo::stop)
        {
            if (_tSigns[i].assigned)
                painter->fillPath(stop(_tSigns[i].pos), QBrush(Qt::red));
            else
                painter->fillPath(stop(_tSigns[i].pos), QBrush(Qt::black));
        }
    }


    pen.setColor(Qt::gray);
    pen.setStyle(Qt::SolidLine);
    painter->setPen(pen);
    for (uint i = 0; i < _sObjs.size(); ++i)
    {
        if (_sObjs[i].kind != conflict::staticObjKind::crosswalk)
            continue;

        painter->drawPath( crossWalk(_sObjs[i].pos, _sObjs[i].tg, _sObjs[i].w, _sObjs[i].l) );
    }


    pen.setColor(Qt::gray);
    painter->setPen(pen);
    for (uint i = 0; i < crossingPoints.size(); ++i)
    {
        qreal r = 0.1 * ct::mToPix;
        painter->drawEllipse(QPointF(ct::mToPix * crossingPoints[i][0], -ct::mToPix * crossingPoints[i][1]), r, r);
    }
}

QPainterPath graphicalRNS::giveWay(const arr2 &o) const
{
    QPainterPath qpp;
    qpp.moveTo(ct::mToPix * (o[0] - 0.5), - ct::mToPix * (o[1] + 0.28867513459481287));
    qpp.lineTo(ct::mToPix * (o[0] + 0.5), - ct::mToPix * (o[1] + 0.28867513459481287));
    qpp.lineTo(ct::mToPix * o[0], -ct::mToPix * (o[1] -0.5773502691896257));

    qpp.lineTo(ct::mToPix * (o[0] - 0.5), - ct::mToPix * (o[1] + 0.28867513459481287));
    qpp.lineTo(ct::mToPix * (o[0] + 0.5), - ct::mToPix * (o[1] + 0.28867513459481287));
    qpp.lineTo(ct::mToPix * o[0], -ct::mToPix * (o[1] -0.5773502691896257));
    return qpp;
}

QPainterPath graphicalRNS::stop(const arr2 &o) const
{
    QPainterPath qpp;

    qpp.moveTo(ct::mToPix * (o[0] - 0.5),                -ct::mToPix * (o[1] + 1.2071067811865475));
    qpp.lineTo(ct::mToPix * (o[0] + 0.5),                -ct::mToPix * (o[1] + 1.2071067811865475));
    qpp.lineTo(ct::mToPix * (o[0] + 1.2071067811865475), -ct::mToPix * (o[1] + 0.5));
    qpp.lineTo(ct::mToPix * (o[0] + 1.2071067811865475), -ct::mToPix * (o[1] - 0.5));

    qpp.lineTo(ct::mToPix * (o[0] + 0.5),                -ct::mToPix * (o[1] - 1.2071067811865475));
    qpp.lineTo(ct::mToPix * (o[0] - 0.5),                -ct::mToPix * (o[1] - 1.2071067811865475));
    qpp.lineTo(ct::mToPix * (o[0] - 1.2071067811865475), -ct::mToPix * (o[1] - 0.5));
    qpp.lineTo(ct::mToPix * (o[0] - 1.2071067811865475), -ct::mToPix * (o[1] + 0.5));

    return qpp;
}

QPainterPath graphicalRNS::crossWalk(const arr2 &c, const arr2 &tg, scalar width, scalar length) const
{
    QPainterPath qpp;

    arr2 n = tg;
    mvf::rotateVectorByAngle(n, 0.5*ct::pi);

    arr2 bp = { c[0] + 0.5*length*tg[0] - 0.5*width*n[0], c[1] + 0.5*length*tg[1] - 0.5*width*n[1]};
    arr2 bs = { c[0] + 0.5*length*tg[0] + 0.5*width*n[0], c[1] + 0.5*length*tg[1] + 0.5*width*n[1]};
    arr2 sp = { c[0] - 0.5*length*tg[0] - 0.5*width*n[0], c[1] - 0.5*length*tg[1] - 0.5*width*n[1]};
    arr2 ss = { c[0] - 0.5*length*tg[0] + 0.5*width*n[0], c[1] - 0.5*length*tg[1] + 0.5*width*n[1]};

    qpp.moveTo( ct::mToPix * bp[0], -ct::mToPix * bp[1] );
    qpp.lineTo( ct::mToPix * bs[0], -ct::mToPix * bs[1] );
    qpp.lineTo( ct::mToPix * ss[0], -ct::mToPix * ss[1] );
    qpp.lineTo( ct::mToPix * sp[0], -ct::mToPix * sp[1] );
    qpp.lineTo( ct::mToPix * bp[0], -ct::mToPix * bp[1] );

    scalar d = 1.5;
    uint parts = length / d;
    d = length / parts;

    for (uint i = 1; i < parts; ++i)
    {
        qpp.moveTo( ct::mToPix * (sp[0] + i*d*tg[0]), -ct::mToPix * (sp[1] + i*d*tg[1]) );
        qpp.lineTo( ct::mToPix * (ss[0] + i*d*tg[0]), -ct::mToPix * (ss[1] + i*d*tg[1]) );
    }

    return qpp;
}


#endif // QT_CORE_LIB
