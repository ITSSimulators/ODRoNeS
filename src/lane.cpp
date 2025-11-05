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

#include <sstream>
#include "lane.h"
#include "xmlUtils.h"
// DEBUG //
#include <fstream>
#include <boost/format.hpp>
#include <tinyxml2.h>


using namespace odrones;

// Static methods:
std::string lane::tSignInfoString(tSignInfo s)
{
    switch(s)
    {
    case tSignInfo::giveWay:
        return "giveWay";
    case tSignInfo::stop:
        return "stop";
    case tSignInfo::unknown:
        return "unknown";
    default:
        return "Unrecognised traffic sign info";
    }
}


std::string lane::signString(sign s)
{
    switch(s)
    {
    case sign::n:
        return "n";
    case sign::p:
        return "p";
    case sign::o:
        return "o";
    default:
        return "unrecognised sign";
    }
}




// Constructors:
lane::lane()
{
    base();
}

lane::lane(const arr2 &origin, const arr2 &dest, scalar width, scalar speed, mvf::shape shp, sign sgn, bool permanent)
{
    base();
    set(origin, dest, width, speed, shp, sgn, permanent);
}

lane::lane(const std::vector<bezier2> &bzr, scalar width, scalar speed, sign sgn, bool permanent)
{
    base();
    initialise(width, speed, mvf::shape::bezier2, sgn, permanent);
    setBezierLines(bzr);
}

lane::lane(const std::vector<bezier3> &bzr, scalar width, scalar speed, sign sgn, bool permanent)
{
    base();
    initialise(width, speed, mvf::shape::bezier3, sgn, permanent);
    setBezierLines(bzr);
}

lane::lane(const std::vector<arr2> &bzrp, mvf::shape s, scalar width,
           scalar speed, sign sgn, bool permanent)
{
    base();
    set(bzrp, s, width, speed, sgn, permanent);

}

void lane::base()
{
    _width = 3;
    _constantWidth = true;
    _speed = 30 * ct::mphToMs;
    _isPermanent = true;

    _kind = kind::tarmac;

    _bbblc = {0, 0};
    _bbtrc = {0, 0};

    _id = -1;
    _nextLane = nullptr;
    _nextLaneSize = 0;
    _prevLane = nullptr;
    _prevLaneSize = 0;
    _portLane = nullptr;
    _starboardLane = nullptr;
    _odrZero = nullptr;
    _length = 0;

    _shape = mvf::shape::unknown;

    _section = nullptr;
    _sectionID = -1;

    _sign = sign::o;

    _odrID = (std::numeric_limits<int>::max)();
    _odrSectionID = -1;
    _odrFwd = true;
    _flippable = true;

    numerical::base();
}

void lane::initialise(scalar width, scalar speed, mvf::shape shp, sign sgn, bool permanent)
{
    _width = width;
    _speed = speed;
    _isPermanent = permanent;
    _shape = shp;
    _sign = sgn;
}

void lane::set(const arr2 &origin, const arr2 &dest, scalar width, scalar speed, mvf::shape shp, sign sgn, bool permanent)
{
    if (isSet())
    {
        std::cerr << "resetting lane: " << getSUID() << std::endl;
        clearMemory();
    }

    initialise(width, speed, shp, sgn, permanent);

    if (shp == mvf::shape::straight)
    {
        _geom.push_back(new straight(origin, dest));
        _length = _geom[0]->length();
        _bbblc = _geom[0]->blc();
        _bbtrc = _geom[0]->trc();
    }

    else if ((shp == mvf::shape::clockwise) || (shp == mvf::shape::counterclockwise))
    {
        _geom.push_back(new arc(origin, dest));
        _length = _geom[0]->length();
        _bbblc = _geom[0]->blc();
        _bbtrc = _geom[0]->trc();
        // setupCurvedLane();
    }

    else
        std::cerr << "[ Error ] we cannot set a lane here that is not straight or arc" << std::endl;

}

void lane::set(const arr2 &origin, const arr2 &dest, const arr2 &centre, scalar width, scalar speed, mvf::shape shp, sign sgn, bool permanent)
{
    if (isSet())
    {
        std::cerr << "resetting lane: " << getSUID() << std::endl;
        clearMemory();
    }


    initialise(width, speed, shp, sgn, permanent);

    _geom.push_back(new arc(origin, dest, centre, shp));
    _length = _geom[0]->length();
    _bbblc = _geom[0]->blc();
    _bbtrc = _geom[0]->trc();
}


void lane::set(const std::vector<bezier2> &bzr, scalar width, scalar speed, sign sgn, bool permanent)
{
    initialise(width, speed, mvf::shape::bezier2, sgn, permanent);
    setBezierLines(bzr);
}

void lane::set(const std::vector<bezier3> &bzr, scalar width, scalar speed, sign sgn, bool permanent)
{
    initialise(width, speed, mvf::shape::bezier3, sgn, permanent);
    setBezierLines(bzr);
}

void lane::set(const std::vector<arr2> &bzrp, mvf::shape s, scalar width, scalar speed, sign sgn, bool permanent)
{
    if (s == mvf::shape::bezier2)
    {
        if (bzrp.size() % 3)
        {
            std::cerr << "[ Error ] configuring lane " << _id << " as an array of bezier2 curves, as we didn't received the right amount of points" << std::endl;
            return;
        }

        uint bzCurves = static_cast<uint>(bzrp.size()) /  3;
        std::vector<bezier2> bz(bzCurves);
        for (uint i = 0; i < bzrp.size(); ++i)
            bz[i].set( bzrp[3*i], bzrp[3*i+1], bzrp[3*i+2] );

        set(bz, width, speed, sgn, permanent);
    }

    else if (s == mvf::shape::bezier3)
    {
        if (bzrp.size() % 4)
        {
            std::cerr << "[ Error ] configuring lane " << _id << " as an array of bezier3 curves, as we didn't received the right amount of points" << std::endl;
            return;
        }

        uint bzCurves = static_cast<uint>(bzrp.size()) /  4;
        std::vector<bezier3> bz(bzCurves);
        for (uint i = 0; i < bzCurves; ++i)
            bz[i].set( bzrp[4*i], bzrp[4*i+1], bzrp[4*i+2], bzrp[4*i+3] );

        set(bz, width, speed, sgn, permanent);
    }
}



void lane::set(const std::vector<Odr::geometry> &odrg, std::vector<Odr::offset> off,
               const std::vector<Odr::offset> &width, const Odr::smaL &odrL, scalar endingS)
{
    _odrID = odrL.odrID;

    _sign = lane::sign::o;
    if (odrL.sign == -1) _sign = lane::sign::n;
    else if (odrL.sign == 1) _sign = lane::sign::p;

    if (!width.empty())
        _width = width[0].a;
    else                // if a lane has width zero, it will be width will be empty here.
        _width = 0;
    _isPermanent = true;
    _shape = mvf::shape::opendrive;

    if (isOdrTransitable(odrL.kind.c_str()))
        _kind = kind::tarmac;
    else if ((odrL.kind.compare(Odr::Kind::Sidewalk) == 0) || (odrL.kind.compare(Odr::Kind::Walking) == 0))
        _kind = kind::pavement;
    else if (odrL.kind.compare(Odr::Kind::None) == 0)
        _kind = kind::none;
    else
        _kind = kind::unknown;

    // store the width and the starting point of the lane section.
    _odrSo = odrL.startingS;
    _odrWidth = width;

    bool geomPrint = false;
    _constantWidth = true;

    _length = 0;
    scalar so = odrL.startingS;
    scalar se = endingS;
    scalar roadSoi = 0;
    for (uint i = 0; i < odrg.size(); ++i)
    {
        if (so >= odrg[i].length - mvf::distPrecision)
        {
            so -= odrg[i].length;
            se -= odrg[i].length;
            roadSoi += odrg[i].length;
            continue;
        }

        scalar soi = so;
        if (so < 0) soi = 0;
        if (mvf::areCloseEnough(se, 0, 1e-8)) se = 0;
        scalar sei = se;
        if (se > odrg[i].length) sei = odrg[i].length;

        if ((off.size() == 1) && (off[0].isConstant()))
        {
            if (odrg[i].g == Odr::Attr::Geometry::line)
                _geom.push_back(new straight(odrg[i], getSignInt(), off[0].a, soi, sei, roadSoi));
            else if (odrg[i].g == Odr::Attr::Geometry::arc)
                _geom.push_back(new arc(odrg[i], getSignInt(), off[0].a, soi, sei, roadSoi));
            else if (odrg[i].g == Odr::Attr::Geometry::paramPoly3)
                _geom.push_back(new paramPoly3(odrg[i], getSignInt(), off[0].a, soi, sei, roadSoi));
            else if (odrg[i].g == Odr::Attr::Geometry::spiral)
                _geom.push_back(new vwSpiral(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
            else if (odrg[0].g == Odr::Attr::Geometry::bezier3)
                _geom.push_back(new vwBezier3(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
            else
            {
                std::cerr << "[ Lane ] Unsupported shape! The code will crash quickly after this." << std::endl;
                continue;
            }
        }
        else
        {
            _constantWidth = false;
            if (odrg[i].g == Odr::Attr::Geometry::line)
                _geom.push_back(new vwStraight(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
            else if (odrg[i].g == Odr::Attr::Geometry::arc)
                _geom.push_back(new vwArc(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
            else if (odrg[i].g == Odr::Attr::Geometry::paramPoly3)
                _geom.push_back(new vwParamPoly3(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
            else if (odrg[i].g == Odr::Attr::Geometry::spiral)
                _geom.push_back(new vwSpiral(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
            else if (odrg[0].g == Odr::Attr::Geometry::bezier3)
                _geom.push_back(new vwBezier3(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
            else
            {
                std::cerr << "[ Lane ] Unsupported shape! The code will crash quickly after this." << std::endl;
                continue;
            }
        }


        _length += _geom.back()->length();

        so -= odrg[i].length;
        se -= odrg[i].length;
        roadSoi += odrg[i].length;

        if (se < mvf::distPrecision) //  || (se - so < mvf::distPrecision))
            break;
    }

    scalar maxSo = _geom.back()->sl0(_geom.back()->length());
    for (uint i = 0; i < _odrWidth.size(); ++i)
    {
        if ((_odrWidth[i].lr == Odr::offset::LR::RL) &&
                (_odrWidth[i].se < maxSo))
            _odrWidth[i].se = maxSo;
    }

    calcBoundingBox();


    for (uint i = 0; i < odrL.speed.size(); ++i)
    {
        if (mvf::areSameValues(odrL.speed[i].value, 0.)) continue;
        _odrSpeed.push_back(odrL.speed[i]);
        _odrSpeed.back().s = sli(odrL.speed[i].s);
    }


    if (geomPrint)
        writeDown();

    return;

}


void lane::convertBezierToParamPoly3(const vwBezier3 *bez, tinyxml2::XMLElement *geometry, tinyxml2::XMLDocument &doc) const
{
    // Read geometry origin  attributes (global position and heading)
    double x0 = geometry->DoubleAttribute("x");
    double y0 = geometry->DoubleAttribute("y");
    double hdg = geometry->DoubleAttribute("hdg");

    // Convert global to local
    auto toLocal = [&](const arr2& pt) {double dx = pt[0] - x0;
    double dy = pt[1] - y0; double cos_h = cos(-hdg); double sin_h = sin(-hdg);
        return arr2{dx * cos_h - dy * sin_h, dx * sin_h + dy * cos_h};};

    // Control points (global coords)
    auto p0_global = bez->l0ControlPoint(3);auto p1_global = bez->l0ControlPoint(2);auto p2_global = bez->l0ControlPoint(1);auto p3_global = bez->l0ControlPoint(0);

    // Convert to local coordinates and assign to polynomial coefficients
    auto p0 = toLocal(p0_global);auto p1 = toLocal(p1_global);auto p2 = toLocal(p2_global);auto p3 = toLocal(p3_global);
    double aU = p0[0]; double bU = 3 * (p1[0] - p0[0]); double cU = 3 * (p2[0] - 2 * p1[0] + p0[0]);double dU = p3[0] - 3 * p2[0] + 3 * p1[0] - p0[0];
    double aV = p0[1];double bV = 3 * (p1[1] - p0[1]);double cV = 3 * (p2[1] - 2 * p1[1] + p0[1]);double dV = p3[1] - 3 * p2[1] + 3 * p1[1] - p0[1];

    auto* xmlPP3 = doc.NewElement("paramPoly3");
    xmlUtils::setAttrDouble(xmlPP3, "aU", aU);
    xmlUtils::setAttrDouble(xmlPP3, "bU", bU);
    xmlUtils::setAttrDouble(xmlPP3, "cU", cU);
    xmlUtils::setAttrDouble(xmlPP3, "dU", dU);
    xmlUtils::setAttrDouble(xmlPP3, "aV", aV);
    xmlUtils::setAttrDouble(xmlPP3, "bV", bV);
    xmlUtils::setAttrDouble(xmlPP3, "cV", cV);
    xmlUtils::setAttrDouble(xmlPP3, "dV", dV);
    xmlPP3->SetAttribute("pRange", "normalized");

    geometry->InsertEndChild(xmlPP3);
}

void lane::writeDown()
{
    std::cout << "lane " << getCSUID()
              << " origin: (" << getOrigin()[0] << ", " << getOrigin()[1] << ")"
              << " _dest: (" << getDestination()[0] << ", " << getDestination()[1] << ")"
              << std::endl;

    std::ofstream f;
    std::string basename = getCSUID();
#if _WINDOWS
    std::replace(basename.begin(), basename.end(), ':', ';'); // we cannot have a filename with colons on Windows...
#endif

    f.open(basename);
    for (uint i = 0; i < _pointsSize; ++i)
    {
        f << boost::format("%10d %12.3f %12.3f") %
                 i % _pointsX[i] % _pointsY[i] << std::endl;
    }
    f.close();

    f.open(basename + ".box");
    f << boost::format("%12.3f %12.3f") %
             _bbblc[0] % _bbblc[1] << std::endl;
    f << boost::format("%12.3f %12.3f") %
             _bbblc[0] % _bbtrc[1] << std::endl;
    f << boost::format("%12.3f %12.3f") %
             _bbtrc[0] % _bbtrc[1] << std::endl;
    f << boost::format("%12.3f %12.3f") %
             _bbtrc[0] % _bbblc[1] << std::endl;
    f << boost::format("%12.3f %12.3f") %
             _bbblc[0] % _bbblc[1] << std::endl;
    f.close();


    f.open(basename + ".geom");
    for (uint i = 0; i < _geom.size(); ++i)
    {
        f << boost::format("%12.3f %12.3f") %
                 _geom[i]->origin()[0] % _geom[i]->origin()[1] << std::endl;
        f << boost::format("%12.3f %12.3f") %
                 _geom[i]->dest()[0] % _geom[i]->dest()[1] << std::endl;
    }
    f.close();

    f.open(basename + ".geom.boxes");
    for (uint i = 0; i < _geom.size(); ++i)
    {
        f << "#" << mvf::shapeString( _geom[i]->shape() ) << std::endl;
        f << boost::format("%12.3f %12.3f") %
                 _geom[i]->blc()[0] % _geom[i]->blc()[1] << std::endl;
        f << boost::format("%12.3f %12.3f") %
                 _geom[i]->blc()[0] % _geom[i]->trc()[1] << std::endl;
        f << boost::format("%12.3f %12.3f") %
                 _geom[i]->trc()[0] % _geom[i]->trc()[1] << std::endl;
        f << boost::format("%12.3f %12.3f") %
                 _geom[i]->trc()[0] % _geom[i]->blc()[1] << std::endl;
        f << boost::format("%12.3f %12.3f") %
                 _geom[i]->blc()[0] % _geom[i]->blc()[1] << std::endl;
        f << std::endl << std::endl;
    }
    f.close();

    f.open(basename + ".geom.numerical");
    std::ofstream ff;
    ff.open(basename + ".geom.S");
    for (uint i = 0; i < _geom.size(); ++i)
    {
        if (!_geom[i]->isNumerical()) continue;
        f << "#" << mvf::shapeString( _geom[i]->shape() ) << std::endl;
        std::vector<arr2> p;
        std::vector<scalar> s;
        if ((_geom[i]->shape() == mvf::shape::vwStraight) ||
            (_geom[i]->shape() == mvf::shape::vwArc) ||
            (_geom[i]->shape() == mvf::shape::vwParamPoly3) ||
            (_geom[i]->shape() == mvf::shape::vwSpiral))

        {
            p = static_cast<vwNumerical*>(_geom[i])->points();
            s = static_cast<vwNumerical*>(_geom[i])->S();
        }
        else if (_geom[i]->shape() == mvf::shape::paramPoly3)
        {
            p = static_cast<paramPoly3*>(_geom[i])->points();
            s = static_cast<paramPoly3*>(_geom[i])->S();
        }

        for (uint j = 0; j < p.size(); ++j)
            f << boost::format("%12.3f %12.3f") %
                     p[j][0] % p[j][1] << std::endl;

        for (uint j = 0; j < s.size(); ++j)
            ff << boost::format("%12.3f") % s[j] << std::endl;
    }
    f.close();
    ff.close();
}

void lane::set(const OneVersion::smaS &sec, uint index)
{
    const OneVersion::smaL *smal = &(sec.lanes[index]);
    _ovID = smal->ovID;

    lane::sign s =lane::sign::o;
    if (smal->dir == 1) s = lane::sign::p;
    else if (smal->dir == -1) s = lane::sign::n;

    initialise(smal->width0, sec.speedLimit, mvf::shape::oneversion, s);

    _kind = kind::tarmac;

    // Having this sort of variable width would be not difficult.
    // It would just mean having an extra step at getPointAfterDistance and the like.
    bool vw = true;
    if ((mvf::areSameValues(smal->curve.centreFunction.b, 0)) &&
            (mvf::areSameValues(smal->curve.leftEdgeFunction.b, 0)) &&
            (mvf::areSameValues(smal->curve.rightEdgeFunction.b, 0)))
        vw = false;
    if (vw)
    {
        std::cout << "Active reference functions are not supported yet!" << std::endl;
        // return;
    }


    _length = 0;
    for (uint i = 0; i < smal->curve.segments.size(); ++i)
    {
        const OneVersion::segment* sgm = &(smal->curve.segments[i]);
        if (sgm->type == OneVersion::SegmentType::straight)
            _geom.push_back(new straight(*sgm, smal->curve.centreFunction.a));
        else if (sgm->type == OneVersion::SegmentType::circular)
            _geom.push_back(new arc(*sgm, smal->curve.centreFunction.a));
        else
        {
            std::cerr << "Unsupported shape!" << std::endl;
        }

        _length += _geom.back()->length();
    }

    calcBoundingBox();



}

bool lane::isOdrFwd() const
{
    return _odrFwd;
}

bool lane::isOdrShapeSupported(mvf::shape s) const
{
    switch(s)
    {
    case mvf::shape::straight:
        return true;
    case mvf::shape::clockwise:
        return true;
    case mvf::shape::counterclockwise:
        return true;
    case mvf::shape::paramPoly3:
        return true;
    case mvf::shape::vwSpiral:
        return true;
    case mvf::shape::vwBezier3:
        return true;
    default:
        return false;
    }
}


bool lane::xmlPlanView(tinyxml2::XMLElement *planView, tinyxml2::XMLDocument &doc) const
{
    if (_odrID != 0) return false;

    // We're in Lane Zero from here on:
    for (uint i = 0; i < _geom.size(); ++i)
    {
        tinyxml2::XMLElement *geometry = doc.NewElement(Odr::Elem::Geometry);
        if (!geometry)
        {
            std::cerr << "[ Error ] Lane::xmlPlanView unable to create geometry element" << std::endl;
            return false;
        }

        xmlUtils::setAttrDouble(geometry, Odr::Attr::S, _geom[i]->roadSo());
        xmlUtils::setAttrDouble(geometry, Odr::Attr::X, _geom[i]->o()[0]);
        xmlUtils::setAttrDouble(geometry, Odr::Attr::Y,  _geom[i]->o()[1]);
        xmlUtils::setAttrDouble(geometry, Odr::Attr::Hdg,
                                std::atan2(_geom[i]->to()[1], _geom[i]->to()[0]));
        xmlUtils::setAttrDouble(geometry, Odr::Attr::Length,
                                _geom[i]->roadSe() - _geom[i]->roadSo());
        if (_geom[i]->shape() == mvf::shape::straight)
        {
            tinyxml2::XMLElement *xmlLine = doc.NewElement(Odr::Elem::Line);
            geometry->InsertEndChild(xmlLine);
        }
        else if (_geom[i]->isArc())
        {
            tinyxml2::XMLElement *xmlArc = doc.NewElement(Odr::Elem::Arc);
            xmlUtils::setAttrDouble(xmlArc, Odr::Attr::Curvature,
                                    (1. / static_cast<arc*>(_geom[i])->radiusOfCurvature()));
            geometry->InsertEndChild(xmlArc);
        }
        else if (_geom[i]->shape() == mvf::shape::paramPoly3)
        {
            tinyxml2::XMLElement *xmlPP3 = doc.NewElement(Odr::Elem::ParamPoly3);
            xmlUtils::setAttrDouble(xmlPP3, Odr::Attr::aU,
                                    (static_cast<paramPoly3*>(_geom[i])->u(0)));
            xmlUtils::setAttrDouble(xmlPP3,Odr::Attr::bU,
                                    (static_cast<paramPoly3*>(_geom[i])->u(1)));
            xmlUtils::setAttrDouble(xmlPP3, Odr::Attr::cU,
                                    (static_cast<paramPoly3*>(_geom[i])->u(2)));
            xmlUtils::setAttrDouble(xmlPP3, Odr::Attr::dU,
                                    (static_cast<paramPoly3*>(_geom[i])->u(3)));

            xmlUtils::setAttrDouble(xmlPP3, Odr::Attr::aV,
                                    (static_cast<paramPoly3*>(_geom[i])->v(0)));
            xmlUtils::setAttrDouble(xmlPP3,Odr::Attr::bV,
                                    (static_cast<paramPoly3*>(_geom[i])->v(1)));
            xmlUtils::setAttrDouble(xmlPP3, Odr::Attr::cV,
                                    (static_cast<paramPoly3*>(_geom[i])->v(2)));
            xmlUtils::setAttrDouble(xmlPP3, Odr::Attr::dV,
                                    (static_cast<paramPoly3*>(_geom[i])->v(3)));

            if (static_cast<paramPoly3*>(_geom[i])->normalised())
                xmlPP3->SetAttribute(Odr::Attr::pRange, Odr::Kind::normalized);
            else
                xmlPP3->SetAttribute(Odr::Attr::pRange, Odr::Kind::arcLength);

            geometry->InsertEndChild(xmlPP3);
        }
        else if (_geom[i]->shape() == mvf::shape::vwSpiral)
        {
            tinyxml2::XMLElement *xmlSpiral = doc.NewElement(Odr::Elem::Spiral);
            xmlUtils::setAttrDouble(xmlSpiral, Odr::Attr::CurvStart,
                                    (static_cast<vwSpiral*>(_geom[i])->l0CurvStart()));
            xmlUtils::setAttrDouble(xmlSpiral, Odr::Attr::CurvEnd,
                                    (static_cast<vwSpiral*>(_geom[i])->l0CurvEnd()));
            geometry->InsertEndChild(xmlSpiral);
        }
        else if (_geom[i]->shape() == mvf::shape::vwBezier3)
        {
            bool writeAsPP3 = false;
            if (writeAsPP3)
            {
                vwBezier3* bez=static_cast<vwBezier3*>(_geom[i]);
                convertBezierToParamPoly3(bez,geometry,doc);
            }
            else
            {
                tinyxml2::XMLElement *xmlBezier = doc.NewElement(Odr::Elem::Bezier3);

                xmlUtils::setAttrDouble(xmlBezier, Odr::Attr::bz0x,
                                        (static_cast<vwBezier3*>(_geom[i])->l0ControlPoint(0)[0] ));
                xmlUtils::setAttrDouble(xmlBezier, Odr::Attr::bz0y,
                                        (static_cast<vwBezier3*>(_geom[i])->l0ControlPoint(0)[1] ));
                xmlUtils::setAttrDouble(xmlBezier, Odr::Attr::bz1x,
                                        (static_cast<vwBezier3*>(_geom[i])->l0ControlPoint(1)[0] ));
                xmlUtils::setAttrDouble(xmlBezier, Odr::Attr::bz1y,
                                        (static_cast<vwBezier3*>(_geom[i])->l0ControlPoint(1)[1] ));
                xmlUtils::setAttrDouble(xmlBezier, Odr::Attr::bz2x,
                                        (static_cast<vwBezier3*>(_geom[i])->l0ControlPoint(2)[0] ));
                xmlUtils::setAttrDouble(xmlBezier, Odr::Attr::bz2y,
                                        (static_cast<vwBezier3*>(_geom[i])->l0ControlPoint(2)[1] ));
                xmlUtils::setAttrDouble(xmlBezier, Odr::Attr::bz3x,
                                        (static_cast<vwBezier3*>(_geom[i])->l0ControlPoint(3)[0] ));
                xmlUtils::setAttrDouble(xmlBezier, Odr::Attr::bz3y,
                                        (static_cast<vwBezier3*>(_geom[i])->l0ControlPoint(3)[1] ));

                geometry->InsertEndChild(xmlBezier);
            }
        }
        else
        {
            return false;
        }
        planView->InsertEndChild(geometry);
    }
    return true;
}

bool lane::xmlLaneAttributesAndLinks(tinyxml2::XMLElement *elem, tinyxml2::XMLDocument &doc, const std::string &type) const
{
    if (!elem) return false;

    elem->SetAttribute(Odr::Attr::Id, _odrID);
    elem->SetAttribute(Odr::Attr::Type, type.c_str());
    elem->SetAttribute(Odr::Attr::Level, Odr::Kind::False);

    tinyxml2::XMLElement* link = doc.NewElement(Odr::Elem::Link);
    if (!link) return false;
    if (_nextLaneSize)
    {
        tinyxml2::XMLElement *successor = doc.NewElement(Odr::Elem::Successor);
        if (!successor) return false;
        successor->SetAttribute(Odr::Attr::Id, _nextLane[0]->odrID());
        link->InsertEndChild(successor);
    }
    if (_prevLaneSize)
    {
        tinyxml2::XMLElement *predecessor = doc.NewElement(Odr::Elem::Predecessor);
        if (!predecessor) return false;
        predecessor->SetAttribute(Odr::Attr::Id, _prevLane[0]->odrID());
        link->InsertEndChild(predecessor);
    }
    elem->InsertEndChild(link);

    return true;
}


bool lane::xmlLaneAttributesAndLinks(tinyxml2::XMLElement *elem, tinyxml2::XMLDocument &doc) const
{
    if (!elem) return false;

    std::string laneType;
    if (_kind == kind::tarmac)
        laneType = Odr::Kind::Driving;
    else if (_kind == kind::pavement)
        laneType = Odr::Kind::Walking;
    else
        laneType = "none";

    return xmlLaneAttributesAndLinks(elem, doc, laneType);
}

bool lane::flipBackwards()
{

    bool isFwd = false; // I know, I know...
    if (isFwd == isOdrFwd()) return true;

    if (!_flippable)
    {
        std::cout << "[ lane ] " << getSUID() << " won't flip because is non-flippable" << std::endl;
        return false;
    }

    if (!isFwd)
    {
        _flippable = false;
        _odrFwd = (!_odrFwd); // false;

        if (_geom.size())
        {
            std::vector<geometry*> tmpG;
            for (int i = static_cast<int>(_geom.size()) - 1; i >= 0; --i)
            {
                size_t it = static_cast<size_t>(i);
                _geom[it]->invert();
                tmpG.push_back(_geom[it]);
            }
            _geom = tmpG;
        }

        uint oldNextLaneSize = _nextLaneSize;
        const lane** oldNextLane = _nextLane;
        _nextLaneSize = _prevLaneSize;
        _nextLane = _prevLane;

        _prevLaneSize = oldNextLaneSize;
        _prevLane = oldNextLane;

        const lane* oldPortLane = _portLane;
        _portLane = _starboardLane;
        _starboardLane = oldPortLane;


        // POINTS: have to be reversed too, so do recalculate:
        if (_pointsSize)
        {
            numerical::zeroPoints();
            numerical::setup();
        }


        // CROSSWALKS:
        // flip start and end of every crosswalk:
        for (uint i = 0; i < _conflicts.size(); ++i)
        {
            scalar dso = _conflicts[i].s - _conflicts[i].so;
            scalar dse = _conflicts[i].se - _conflicts[i].s;
            _conflicts[i].s = _length - _conflicts[i].s;
            _conflicts[i].so = _conflicts[i].s - dso;
            _conflicts[i].se = _conflicts[i].s + dse;
        }
        // and reorder the crosswalks:
        std::vector<conflict> tmp = _conflicts;
        for (uint i = 0; i < _conflicts.size(); ++i)
            _conflicts[i] = tmp[tmp.size()-1-i];

    }

    else
    {
        std::cerr << "[ lane ] odr change back to positive is not allowed." << std::endl;
        return false;
    }

    return true;
}

bool lane::isFlippable() const
{
    return _flippable;
}

void lane::lockFlippable()
{
    _flippable = false;
}


void lane::setBezierLines(const std::vector<bezier2> &bzr)
{
    if (isSet())
    {
        std::cerr << "lane: " << _id << " was already set; not proceeding" << std::endl;
        return;
    }

    _length = 0;
    for (uint i = 0; i < bzr.size(); ++i)
    {
        _geom.push_back(new bezier2(bzr[i]));
        _length += _geom[i]->length();
    }

    calcBoundingBox();
}

void lane::numericalSetup()
{

    scalar ds = numerical::defaultDs(_length);
    uint size = 1 + static_cast<uint>(std::round(_length / ds));
    numerical::clearMemory();
    numerical::initialise(ds, size);
    if (numerical::setup())
        std::cout << "[ Error ] on " << getCSUID() << " in configuring numerical for the top class" << std::endl;

}

void lane::setBezierLines(const std::vector<bezier3> &bzr)
{
    if (isSet())
    {
        std::cerr << "lane: " << _id << " was already set; not proceeding" << std::endl;
        return;
    }

    _length = 0;
    for (uint i = 0; i < bzr.size(); ++i)
    {
        _geom.push_back(new bezier3(bzr[i]));
        _length += _geom[i]->length();
    }


    calcBoundingBox();

}

void lane::appendBezierLines(const std::vector<bezier2> &bzr)
{
    for (uint i = 0; i < bzr.size(); ++i)
    {
        _geom.push_back(new bezier2(bzr[i]));
        _length += _geom[i]->length();
    }

    calcBoundingBox();
}



lane::~lane()
{
    clearMemory();
    numerical::clearMemory();
}

void lane::clearMemory()
{
    if (hasNextLane())
    {
        delete[] _nextLane;
        _nextLane = nullptr;
        _nextLaneSize = 0;
    }
    if (hasPrevLane())
    {
        delete[] _prevLane;
        _prevLane = nullptr;
        _prevLaneSize = 0;
    }
    for (uint i = 0; i < _geom.size(); ++i)
        delete _geom[i];
    _geom.clear();

    numerical::clearMemory();

    _conflicts.clear();
    _tSigns.clear();

    _odrSpeed.clear();
    _odrWidth.clear();

    base();
}


lane::lane(const lane& t)
{
    assignInputLaneToThis(t);
}


lane& lane::operator=(const lane& t)
{
    clearMemory();
    assignInputLaneToThis(t);
    return *this;
}


void lane::assignInputLaneToThis(const lane &t)
{
    _id = t._id;
    _width = t._width;
    _constantWidth = t._constantWidth;
    _length = t._length;
    _speed = t._speed;
    _isPermanent = t._isPermanent;
    _kind = t._kind;

    _bbblc = {t._bbblc[0], t._bbblc[1]};
    _bbtrc = {t._bbtrc[0], t._bbtrc[1]};

    if (t.hasNextLane())
    {
        _nextLaneSize = t._nextLaneSize;
        _nextLane = new const lane*[_nextLaneSize];
        for (size_t i=0; i < _nextLaneSize; ++i)
            _nextLane[i] = t._nextLane[i];
    }
    else
    {
        _nextLaneSize = 0;
        _nextLane = nullptr;
    }
    if (t.hasPrevLane())
    {
        _prevLaneSize = t._prevLaneSize;
        _prevLane = new const lane*[_prevLaneSize];
        for (size_t i = 0; i < _prevLaneSize; ++i)
            _prevLane[i] = t._prevLane[i];
    }
    else
    {
        _prevLane = nullptr;
        _prevLaneSize = 0;
    }


    // Geometries
    for (uint i = 0; i < t._geom.size(); ++i)
    {
        if (t._geom[i]->shape() == mvf::shape::straight)
            _geom.push_back(new straight(*(dynamic_cast<straight*>(t._geom[i]))));
        else if (t._geom[i]->isArc())
            _geom.push_back(new arc(*(dynamic_cast<arc*>(t._geom[i]))));
        else if (t._geom[i]->shape() == mvf::shape::bezier2)
            _geom.push_back(new bezier2(*(dynamic_cast<bezier2*>(t._geom[i]))));
        else if (t._geom[i]->shape() == mvf::shape::bezier3)
            _geom.push_back(new bezier3(*(dynamic_cast<bezier3*>(t._geom[i]))));

        else if (t._geom[i]->shape() == mvf::shape::vwStraight)
            _geom.push_back(new vwStraight(*(dynamic_cast<vwStraight*>(t._geom[i]))));
        else if (t._geom[i]->shape() == mvf::shape::vwArc)
            _geom.push_back(new vwArc(*(dynamic_cast<vwArc*>(t._geom[i]))));
        else if (t._geom[i]->shape() == mvf::shape::vwParamPoly3)
            _geom.push_back(new vwParamPoly3(*(dynamic_cast<vwParamPoly3*>(t._geom[i]))));
        else if (t._geom[i]->shape() == mvf::shape::vwSpiral)
            _geom.push_back(new vwSpiral(*(dynamic_cast<vwSpiral*>(t._geom[i]))));
        else if (t._geom[i]->shape() == mvf::shape::vwBezier3)
            _geom.push_back(new vwBezier3(*dynamic_cast<vwBezier3*>(t._geom[i])));

        else
            std::cout << "[ WARNING ] lane::assignInputToThis doesn't know about this geometry" << std::endl;
    }
    _odrSo = t._odrSo;
    _odrWidth = t._odrWidth;
    _odrSpeed = t._odrSpeed;
    _odrZero = t._odrZero;


    _portLane = t._portLane;
    _starboardLane = t._starboardLane;
    _section = t._section;
    _sectionID = t._sectionID;
    _shape = t._shape;

    _sign = t._sign; // travel direction.
    _tSigns = t._tSigns; // traffic signs.
    _conflicts = t._conflicts; // crosswalks.

    _odrID = t._odrID;
    _odrSectionID = t._odrSectionID;
    _odrFwd = t._odrFwd;
    _flippable = t._flippable;

    numerical::assignInputToThis(t);
}



void lane::setPrevLane(const lane *l)
{
    if (isPrevLane(l)) return;
    // if (isNextLane(l)) return;

    if (_prevLaneSize == 0)
        _prevLane = new const lane*[1];
    else
    {
        // check that l was previously not there:
        for (uint i = 0; i < _prevLaneSize; ++i)
            if (_prevLane[i]->isSameLane(l)) return;

        // now carry on with the assignment:
        const lane **tmp = new const lane*[_prevLaneSize];
        for (uint i = 0; i < _prevLaneSize; ++i)
            tmp[i] = _prevLane[i];
        delete[] _prevLane;
        _prevLane = new const lane*[_prevLaneSize + 1];
        for (uint i = 0; i < _prevLaneSize; ++i)
            _prevLane[i] = tmp[i];
        delete[] tmp;
    }
    _prevLane[_prevLaneSize] = l;
    _prevLaneSize += 1;

    // Curved lanes used to need a previous lane to be configured:
    if ((isArc()) && (mvf::areSameValues(_length,0)))
    {
        if (dynamic_cast<arc*>(_geom[0])->pending()) // setupCurvedLane();
        {
            dynamic_cast<arc*>(_geom[0])->setTo(_prevLane[0]->getTangentInPoint(_geom[0]->origin()));
            _length = _geom[0]->length();
            _bbblc = _geom[0]->blc();
            _bbtrc = _geom[0]->trc();
        }
    }
}

/*
void lane::removeNextLane(const lane *l)
{
    std::cout << "remove " << l->getCSUID() << " from next lanes in " << getCSUID() << std::endl;
    return;

    int rmNdx = -1;
    for (uint i = 0; i < _nextLaneSize; ++i)
    {
        if (_nextLane[i]->isCSUID(l->getCSUID()))
        {
            rmNdx = i;
            break;
        }
    }

    if (rmNdx < 0) return;

    for (uint i = rmNdx; i < _nextLaneSize -1; ++i)
        _nextLane[i] = _nextLane[i+1];

    _nextLane[_nextLaneSize-1] = nullptr;

    _nextLaneSize -= 1;
}

void lane::removeNextLane(lane *l, bool crosslink)
{
    removeNextLane(l);

    if (crosslink)
        l->removePrevLane(this);
}


void lane::removePrevLane(const lane *l)
{
    std::cout << "remove " << l->getCSUID() << " from prev lanes in " << getCSUID() << std::endl;
    return;
    int rmNdx = -1;
    for (uint i = 0; i < _prevLaneSize; ++i)
    {
        if (_prevLane[i]->isCSUID(l->getCSUID()))
        {
            rmNdx = i;
            break;
        }
    }

    if (rmNdx < 0) return;

    for (uint i = rmNdx; i < _prevLaneSize -1; ++i)
        _prevLane[i] = _prevLane[i+1];

    _prevLane[_prevLaneSize-1] = nullptr;

    _prevLaneSize -= 1;
}


void lane::removePrevLane(lane *l, bool crosslink)
{
    removePrevLane(l);

    if (crosslink)
        l->removeNextLane(this);
}
*/


void lane::setPrevLane(lane *l, bool crosslink)
{
    setPrevLane(l);

    // Now add this lane as a next lane of l:
    if ( (_isPermanent) && ( (!isCrosswalk())) && (crosslink) )
         l->setNextLane(this, false);
}



void lane::setNextLane(const lane *l)
{
    if (isNextLane(l)) return;
    // if (isPrevLane(l)) return;

    if (_nextLaneSize == 0)
       _nextLane = new const lane*[1];
    else
    {
        // check that l was previously not there:
        for (uint i = 0; i < _nextLaneSize; ++i)
            if (_nextLane[i]->isSameLane(l)) return;

        // now carry on with the assignment:
       const lane **tmp = new const lane*[_nextLaneSize];
       for (uint i = 0; i < _nextLaneSize; ++i)
         tmp[i] = _nextLane[i];
       delete[] _nextLane;
       _nextLane = new const lane*[_nextLaneSize + 1];
       for (uint i = 0; i < _nextLaneSize; ++i)
         _nextLane[i] = tmp[i];
       delete[] tmp;
    }
    _nextLane[_nextLaneSize] = l;
    _nextLaneSize += 1;
}

void lane::setNextLane(lane *l, bool crosslink)
{
    setNextLane(l);

    // Now add this lane as a previous lane of l:
    if ((_isPermanent) && ((!isCrosswalk()) && (crosslink)))
        l->setPrevLane(this, false);
}


void lane::setPrevLane(uint ndx, lane* l)
{
    if (ndx >= _prevLaneSize)
    {
        std::cout << "[ Error ] there is not enough memory booked for lane::setPrevLane to keep track of a lane in: " << ndx;
        return;
    }

    _prevLane[ndx] = l;
}


void lane::setNextLane(uint ndx, lane* l)
{
    if (ndx >= _nextLaneSize)
    {
        std::cout << "[ Error ] there is not enough memory booked for lane::setNextLane to keep track of a lane in: " << ndx;
        return;
    }

    _nextLane[ndx] = l;
}



bool lane::isPermanent() const
{
    return _isPermanent;
}

void lane::permanent(bool p)
{
    _isPermanent = p;
}

bool lane::hasDefinedSign() const
{
    if (_sign == sign::o) return false;
    return true;
}

lane::sign lane::getSign() const
{
    return _sign;
}

std::string lane::getSignString() const
{
    return signString(_sign);
}

int lane::getSignInt() const
{
    if (_sign == sign::p) return 1;
    else if (_sign == sign::n) return -1;
    else return 0;
}

bool lane::isSameSign(sign s) const
{
    if (_sign == sign::o) return false;
    else if (_sign == s) return true;
    return false;
}

bool lane::isSameSign(const lane *l) const
{
    return isSameSign(l->_sign);
}

lane::sign lane::invertSign(sign s)
{
    if (s == sign::n) return sign::p;
    else if (s == sign::p) return sign::n;
    return s;
}

void lane::setPortLane(const lane *l)
{
    _portLane = l;
}

void lane::setStarboardLane(const lane *l)
{
    _starboardLane = l;
}


void lane::setSection(section &s)
{
    _section = &s;
}


const lane* lane::getNextLane() const
{
    return getNextLane(0);
}

const lane* lane::getNextLane(uint idx) const
{
    if (idx < _nextLaneSize ) return _nextLane[idx];
    return nullptr;
}

std::tuple<const lane**, size_t> lane::getNextLanes() const
{
    return std::make_tuple(_nextLane, _nextLaneSize);
}

size_t lane::getNextLaneSize() const
{
    return _nextLaneSize;
}

const lane* lane::getPrevLane() const
{
    return getPrevLane(0);
}

const lane* lane::getPrevLane(uint idx) const
{
    if (idx < _prevLaneSize) return _prevLane[idx];
    return nullptr;
}

std::tuple<const lane**, size_t> lane::getPrevLanes() const
{
    return std::make_tuple(_prevLane, _prevLaneSize);
}

size_t lane::getPrevLaneSize() const
{
    return _prevLaneSize;
}

const lane* lane::getPortLane() const
{
    return _portLane;
}

const lane* lane::getPortLaneSD() const
{
    if (!_portLane) return nullptr;
    if (isSameSign(_portLane)) return _portLane;
    return nullptr;
}

const lane* lane::getStarboardLane() const
{
    return _starboardLane;
}

const lane* lane::getStarboardLaneSD() const
{
    if (!_starboardLane) return nullptr;
    if (isSameSign(_starboardLane)) return _starboardLane;
    return nullptr;
}


mvf::side lane::getMergeSide() const
{
    for (uint i = 0; i < _conflicts.size(); ++i)
    {
        if (_conflicts[i].k == conflict::kind::mergePort) return mvf::side::port;
        else if (_conflicts[i].k == conflict::kind::mergeStarboard) return mvf::side::starboard;
    }
    return mvf::side::bow;
}

const lane* lane::getMergeLane() const
{
    for (uint i = 0; i < _conflicts.size(); ++i)
    {
        if (conflict::isMerge(_conflicts[i].k))
        {
            if (_conflicts[i].k == conflict::kind::mergePort)
                return _portLane;
            else if (_conflicts[i].k == conflict::kind::mergeStarboard)
                return _starboardLane;
        }
    }
    return nullptr;
}

void lane::getOrigin(arr2 &o) const
{
    o = _geom[0]->origin();
}

arr2 lane::getOrigin() const
{
    return _geom[0]->origin();
}

void lane::getDestination(arr2 &d) const
{
    d = _geom.back()->dest();
}

arr2 lane::getDestination() const
{
    return _geom.back()->dest();
}

arr2 lane::getTheClosestExtreme(const arr2 &p) const
{
    if (unsafeDistanceFromTheBoL(p) < 0.5 * _length) return _geom[0]->origin();
    return _geom.back()->dest();
}

void lane::getTo(arr2 &to) const
{
    if (!_geom.size()) return;
    to = _geom[0]->to();
}

arr2 lane::getTo() const
{
    if (_geom.size()) return _geom[0]->to();
    return {0.,0.};
}



scalar lane::getCurvature(const arr2 &p) const
{
    if (_geom.size() == 1)
        return _geom[0]->getCurvature(p);
    else if (_geom.size() > 1)
    {
        int lineIdx = getGeometryIndex(p);
        if (lineIdx < 0)
        {
            std::cerr << "getCurvature couldn't get a valid index for p: (" << p[0] << ", " << p[1] << ")" << std::endl;
            return 0;
        }
        return _geom[lineIdx]->getCurvature(p);
    }
    std::cerr << "[ Error ] lane::getRadiusOfCurvature - unrecognised shape" << std::endl;
    return -1;
}


scalar lane::getLength() const
{
    return _length;
}

mvf::shape lane::getShape(uint i) const
{
    if (_geom.size() > i) return _geom[i]->shape();
    else return mvf::shape::unknown;
}

std::string lane::getShapeString(uint i) const
{
    return mvf::shapeString(getShape(i));
}

std::string lane::getShapeString() const
{
    if (_geom.size() > 1) return "vector";
    else return getShapeString(0);
}

std::string lane::getShapesString() const
{
    std::string shp = mvf::shapeString(getShape(0));
    for (uint i = 1; i < _geom.size(); ++i)
        shp += " - " + mvf::shapeString(getShape(i));
    return shp;
}

uint lane::getGeometrySize() const
{
    return static_cast<uint>(_geom.size());
}


const std::vector<geometry*> lane::geometries() const
{
    return _geom;
}

/*
std::vector<std::unique_ptr<geometry>> lane::getGeometries() const
{
    std::vector<std::unique_ptr<geometry>> v;
    for (size_t i = 0; i < _geom.size(); ++i)
    {
        if (_geom[i]->shape() == mvf::shape::straight)
            v.push_back(std::unique_ptr<geometry>(new straight(*(dynamic_cast<straight*>(_geom[i]))) ));
        else if (_geom[i]->isArc())
            v.push_back(std::unique_ptr<geometry>(new arc(*(dynamic_cast<arc*>(_geom[i]))) ));
        else if (_geom[i]->shape() == mvf::shape::bezier2)
            v.push_back(std::unique_ptr<geometry>(new bezier2(*(dynamic_cast<bezier2*>(_geom[i]))) ));
        else if (_geom[i]->shape() == mvf::shape::bezier3)
            v.push_back(std::unique_ptr<geometry>(new bezier3(*(dynamic_cast<bezier3*>(_geom[i]))) ));
        else if (_geom[i]->shape() == mvf::shape::vwStraight)
            v.push_back(std::unique_ptr<geometry>(new vwStraight(*(dynamic_cast<vwStraight*>(_geom[i]))) ));
        else if (_geom[i]->shape() == mvf::shape::vwArc)
            v.push_back(std::unique_ptr<geometry>(new vwArc(*(dynamic_cast<vwArc*>(_geom[i]))) ));
        else if (_geom[i]->shape() == mvf::shape::vwParamPoly3)
            v.push_back(std::unique_ptr<geometry>(new vwParamPoly3(*(dynamic_cast<vwParamPoly3*>(_geom[i]))) ));
        else if (_geom[i]->shape() == mvf::shape::vwSpiral)
            v.push_back(std::unique_ptr<geometry>(new vwSpiral(*(dynamic_cast<vwSpiral*>(_geom[i]))) ));
        else
            std::cout << "[ WARNING ] lane::getGeometries() - Unknown shape" << std::endl;
    }
    return v;

}
*/

scalar lane::maxSo() const
{
    if (!isOpenDrive())
        return _length;

    scalar maxSo = _odrWidth.back().se;
    for (uint i = 0; i < _odrWidth.size(); ++i)
    {
        if (maxSo < _odrWidth[i].se)
            maxSo = _odrWidth[i].se;
    }
    return maxSo;
}

bool lane::isSet() const
{
    if (!_geom.size()) return false;
    for (uint i = 0; i < _geom.size(); ++i)
        if (!_geom[i]->ready()) return false;
    return true;
}

int lane::getID() const
{
    return _id;
}

int lane::odrSectionID() const
{
    if (!isOpenDrive()) return -1;
    return _odrSectionID;
}

int lane::odrID() const
{
    return _odrID;
}

OneVersion::OVID lane::ovID() const
{
    return _ovID;
}

void lane::setOVID(OneVersion::OVID id)
{
    _ovID = id;
}

void lane::setID(int id)
{
    _id = id;
}

void lane::setSectionID(int sectionID)
{
    _sectionID = sectionID;
}

void lane::setOdrSectionID(int odrSectionID)
{
    _odrSectionID = odrSectionID;
}

int lane::getSectionID() const
{
    return _sectionID;
}

std::string lane::sUID(int sID, int lID)
{
    return std::to_string(sID) + ":" + std::to_string(lID);
}

std::string lane::getCSUID() const
{
    if  (isOpenDrive()) return getSUID() + " (" + getOdrSUID() + ")";
    else if (isOneVersion()) return getSUID() + " (" + getOVSUID() + ")";
    return getSUID();
}

std::string lane::getOdrSUID() const
{
    if (!isOpenDrive()) return "";
    return sUID(_odrSectionID, _odrID);
}

std::string lane::getOVSUID() const
{
    if (!isOneVersion()) return "";
    return _ovID.to_string();
}

std::string lane::getSUID() const
{
    return sUID(_sectionID, _id);
}

bool lane::isSUID(std::string name) const
{
    if (!name.compare(getSUID())) return true;
    return false;
}

bool lane::isCSUID(std::string name) const
{
    if (!name.compare(getCSUID())) return true;
    return false;
}

bool lane::isOdrSUID(std::string name) const
{
    if (!name.compare(getOdrSUID())) return true;
    return false;
}

section* lane::getSection() const
{
    return _section;
}


bool lane::isArc() const
{
    return isArc(_shape);
}

bool lane::isArc(mvf::shape s) const
{
    if ( (s == mvf::shape::clockwise) || (s == mvf::shape::counterclockwise)) return true;
    else return false;
}



bool lane::isOpenDrive() const
{
    if (_shape == mvf::shape::opendrive) return true;
    return false;
}

bool lane::isOneVersion() const
{
    if (_shape == mvf::shape::oneversion) return true;
    return false;
}

void lane::getTangentInPoint(arr2 &t, const arr2 &p) const
{
    if (_geom.size() == 1)
        t = _geom[0]->getTangentInPoint(p);
    else if (_geom.size() > 1)
    {
        int lineIdx = getGeometryIndex(p);
        t = _geom[lineIdx]->getTangentInPoint(p);
    }
    return;
}

arr2 lane::getTangentInPoint(const arr2 &p) const
{
    arr2 t;
    getTangentInPoint(t, p);
    return t;
}

//! THIS IS QtHEADING!!!
scalar lane::getQtHeadingInPoint(const arr2 &p) const
{
    arr2 t;
    getTangentInPoint(t, p);
    // Qt Heading means - angle and swapping x for y:
    return - constants::rad2deg * std::atan2(t[0], t[1]);
}



bool lane::isPointOnLane(const arr2 &p, scalar tol) const
{

    if (_geom.size() == 1)
        return _geom[0]->isPointHere(p);

    else if (_geom.size() > 1)
    {
        for (uint i = 0; i < _geom.size(); ++i)
            if (_geom[i]->isPointHere(p)) return true;
    }
    return false;
}


int lane::getGeometryIndex(const arr2 &p) const
{
    std::vector<int> psb;
    for (uint i = 0; i < _geom.size(); ++i)
        if (mvf::isPointInBoxBLcTRcTol(p, _geom[i]->blc(), _geom[i]->trc(), odrTol))
            psb.push_back(i);

    if (psb.empty())
        return -1;

    else if (psb.size() == 1)
        return psb[0];


    // Otherwise, try projecting it:
    scalar dmin2 = 1e4;
    constexpr scalar dthreshold2 = 25e-4;
    int idx = -1;
    for (uint i = 0; i < _geom.size(); ++i)
    {
        if (std::find(psb.begin(), psb.end(), i) == psb.end())
            continue;


        arr2 prj = _geom[i]->projectPointHere(p);
        scalar di2 = mvf::sqrDistance(prj, p);
        if (di2 < dmin2)
        {
            dmin2 = di2;
            idx = static_cast<int>(i);
        }
    }

    return idx;

    /*
    bool fast = true;
    if (fast) // Fast and slightly insecure
    {
        for (uint i = 0; i < _geom.size(); ++i)
            if (mvf::isPointInBoxBLcTRcTol(p, _geom[i]->blc(), _geom[i]->trc(), odrTol))
                return i;

        return -1;
    }


    // Slow but safe
    // If the point is exactly on the lane, find the right bit:
    for (uint i = 0; i < _geom.size(); ++i)
        if (_geom[i]->isPointHere(p)) return static_cast<int>(i);

    // Otherwise, try projecting it:
    scalar dmin2 = 1e4;
    constexpr scalar dthreshold2 = 25e-4;
    int idx = -1;
    for (uint i = 0; i < _geom.size(); ++i)
    {
        if (!mvf::isPointInBoxBLcTRcTol(p, _geom[i]->blc(), _geom[i]->trc(), odrTol))
            continue;

        arr2 prj = _geom[i]->projectPointHere(p);
        scalar di2 = mvf::sqrDistance(prj, p);
        if (di2 < dmin2)
        {
            dmin2 = di2;
            idx = static_cast<int>(i);
        }
    }

    if (dmin2 < dthreshold2)
        return idx;

    return -1;
    */
}

int lane::getGeometryIndex(scalar d) const
{
    if (!_geom.size()) return -1;

    constexpr scalar accuracy = 1e-6;

    if (d < 0)
    {
        if (mvf::areCloseEnough(d, 0, accuracy))
            d = 0;
        else
            return -1;
    }
    else if (d > _length)
    {
        if (mvf::areCloseEnough(d, _length, accuracy))
            d = _length;
        else
            return -1;
    }

    scalar l = 0;
    for (uint i = 0; i < _geom.size(); ++i)
    {
        l += _geom[i]->length();
        if (d <= l) return i;
    }

    std::cerr << "[ Error ] lane::getGeomIndexForDistance failed." << std::endl;
    return -1;
}


bool lane::getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const
{
    if (_geom.size() == 1)
    {
        if (!_geom[0]->getPointAfterDistance(p, o, d)) return false;
    }
    else if (_geom.size() > 1)
    {
        int lineIdx = getGeometryIndex(o);
        if (lineIdx < 0)
        {
            std::cerr << "lane::getPointAfterDistance opendrive: o " << o[0] << " : " << o[1] << " is not in this lane" << std::endl;
            return false;
        }
        arr2 m = o;
        while (!_geom[lineIdx]->getPointAfterDistance(p, m, d))
        {
            if (static_cast<size_t>(lineIdx) + 1 == _geom.size()) return false; // we're at the end of the lines.
            if (mvf::areSamePoints(m, _geom[lineIdx]->origin()))
                d -= _geom[lineIdx]->length();
            else
                d -= _geom[lineIdx]->distanceToTheEoL(m);
            lineIdx += 1;
            m = _geom[lineIdx]->origin();
        }
        return true;
    }

    if (isPointOnLane(p)) return true;

    return false;
}

bool lane::getPointAtDistance(arr2 &p, scalar s) const
{
    return getPointAfterDistance(p, _geom[0]->origin(), s);
}


scalar lane::unsafeDistanceToTheEoL(const arr2 &p) const
{
    scalar distance = 0;
    if (_geom.size() == 1)
        distance = _geom[0]->distanceToTheEoL(p);
    else if (_geom.size() > 1)
    {
        int lineIdx = getGeometryIndex(p);
        distance = _geom[lineIdx]->distanceToTheEoL(p);
        for (uint i = lineIdx + 1; i < _geom.size(); ++i)
            distance += _geom[i]->length();
    }
    return distance;
}


scalar lane::unsafeDistanceFromTheBoL(const arr2 &p) const
{
    return _length - unsafeDistanceToTheEoL(p);
}


// TEST
bool lane::getPointWithOffset(arr2 &p, const arr2 &o, scalar loff) const
{
    arr2 v = getTangentInPoint(o);
    v = {-v[1], v[0]};
    p = {o[0] + loff * v[0], o[1] + loff * v[1]};
    return true;
}

bool lane::getPointWithOffset(arr2 &p, scalar s, scalar loff) const
{
    arr2 o;
    if (!getPointAtDistance(o, s))
        return false;
    return getPointWithOffset(p, o, loff);
}



void lane::nSetupPointsXYUniformly(scalar ds)
{
    /* Get the _points from the geometries and do the interpolation here */
    _pointsX[0] = _geom[0]->origin()[0]; // _origin[0];
    _pointsY[0] = _geom[0]->origin()[1]; // _origin[1];

    scalar s = ds;
    uint ndx = 1;
    for (uint i = 0; i < _geom.size(); ++i)
    {
        while (s < _geom[i]->length())
        {
            arr2 p;
            if (_geom[i]->isNumerical()) // (shp == mvf::shape::paramPoly3) || (shp == mvf::shape::vwStraight))
            {
                numerical *nm = dynamic_cast<numerical*>(_geom[i]);
                p = nm->interpolate(s);
            }
            else // if ( (_geom[i]->isArc()) || (shp == mvf::shape::straight))
                _geom[i]->getPointAtDistance(p, s); // _geom[i]->getPointAfterDistance(p, _geom[i]->origin(), s);

            _pointsX[ndx] = p[0];
            _pointsY[ndx] = p[1];
            s += ds;
            ndx += 1;
            if (ndx == _pointsSize) break;
        }
        s -= _geom[i]->length();
        if (ndx == _pointsSize) break;
    }

    if (ndx < _pointsSize -1)
    {
        std::cout << "[ WARNING! ] we changed _pointsSize: " << _pointsSize << " to " << ndx + 1 << std::endl;
        _pointsSize = ndx + 1;
    }

    _pointsX[_pointsSize -1] = _geom.back()->dest()[0]; // _dest[0];
    _pointsY[_pointsSize -1] = _geom.back()->dest()[1]; // _dest[1];

}

std::vector<arr2> lane::getIntersectionPoints(lane *l)
{
    // In this case we know it is odr vs odr, and thus we will do numerically with the following approach
    // 1 - Check whether the two bounding boxes do overlap;
    // 2 - if not, return no points;
    // 3 - If overlapping split this into this.1 and this.2 and l into l.1 and l2.
    // 4 - Check the overlapping between the pairs this vs l
    // 5 - Keep the overlapping boxes, and keep halving the curves until every intersection has been found.

    std::vector<arr2> intersections;

    if (!mvf::boxesOverlap(_bbblc, _bbtrc, l->_bbblc, l->_bbtrc)) return intersections;

    if (!numerical::isSet())
        numericalSetup();

    if (!l->numerical::isSet())
        l->numericalSetup();

    mvf::numericalIntersections(intersections, _pointsX, _pointsY, 0, _pointsSize -1, l->_pointsX, l->_pointsY, 0, l->_pointsSize -1);

    return intersections;
}





bool lane::getIntersectionPointFromOT(arr2 &p, const arr2 &o, const arr2 &t) const
{
    bool destSet = false;
    if (_geom.size() > 0)
    {
        for (size_t i = 0; i < _geom.size(); ++i)
            if (_geom[i]->getIntersectionPointFromOT(p, o, t)) return true;
        return false;
    }

    return destSet;
}



arr2 lane::projectPointOntoLane(const arr2 &o) const
{
    arr2 p;
    projectPointOntoLane(p, o);
    return p;
}

bool lane::projectPointOntoLane(arr2 &p, const arr2 &o) const
{
    if (_geom.size() == 1)
    {
        arr2 q = o;
        p = _geom[0]->projectPointHere(q);
    }
    else if (_geom.size() > 1)
    {
        // 1 - Find the best geometries first:
        scalar d2 = 1e12;
        int interval = 0;  // 0 = origin, 1 = centre.
        int gndx = 0;
        for (uint i = 0; i < _geom.size(); ++i)
        {
            scalar d2i = mvf::sqrDistance(o, _geom[i]->origin());
            if (d2i < d2)
            {
                d2 = d2i;
                interval = 0;
                gndx = i;
            }

            arr2 halfway;
            _geom[i]->getPointAtDistance(halfway, 0.5 * _geom[i]->length());
            d2i = mvf::sqrDistance(o, halfway);
            if (d2i < d2)
            {
                d2 = d2i;
                interval = 1;
                gndx = i;
            }
        }
        scalar d2e = mvf::sqrDistance(o, _geom.back()->dest());
        if (d2e < d2)
        {
            gndx = _geom.size() - 1;
            interval = 1;
        }

        if ((interval == 1) || (gndx == 0))
            p = _geom[gndx]->projectPointHere(o);
        else
        {
            arr2 q1 = _geom[gndx]->projectPointHere(o);
            arr2 q2 = _geom[gndx-1]->projectPointHere(o);
            if (mvf::sqrDistance(q1, o) < mvf::sqrDistance(q2, o))
                p = q1;
            else
                p = q2;
        }


        /*
        // 2 - Brute force - project on every geometry:
        for (uint i = 0; i < _geom.size(); ++i)
        {
            arr2 q = _geom[i]->projectPointHere(o);
            scalar d2i = mvf::sqrDistance(q, o);
            if (d2i < d2)
            {
                d2 = d2i;
                p = q;
            }
        }
        */

        return true;
    /*  That used to be a check for the Bezier lanes. Not sure whether we should just ditch it.
        mvf::normalise(tg);
        mvf::tangent(nrm, o, p);
        if ( (std::fabs(tg[0]*nrm[0] + tg[1]*nrm[1]) < 2e-1) || (d2 < 2.5e-3) ) return true; // that's a numerical solution...
    */
    }

    return true;
}


bool lane::getBoundingBox(arr2 &bl, arr2 &tr) const
{
    bl = {_bbblc[0], _bbblc[1]};
    tr = {_bbtrc[0], _bbtrc[1]};
    return true;
}

bool lane::calcBoundingBox()
{
    if (_geom.size())
    {
        _bbblc = _geom[0]->blc();
        _bbtrc = _geom[0]->trc();
        for (uint i = 1; i < _geom.size(); ++i)
            mvf::increaseBoxWithBox(_bbblc, _bbtrc, _geom[i]->blc(), _geom[i]->trc());
        return true;
    }
    return false;
}


bool lane::isSameLane(const lane *l) const
{
    if (!l) return false;
    if ((_id == l->_id) && (_sectionID == l->_sectionID)) return true;
    return false;
}

bool lane::isNextLane(const lane *l) const
{
    if (!l) return false;

    for (uint i = 0; i < _nextLaneSize; ++i)
        if (l->isSameLane(_nextLane[i])) return true;

    return false;
}

bool lane::isPrevLane(const lane *l) const
{
    if (!l) return false;

    for (uint i = 0; i < _prevLaneSize; ++i)
        if (l->isSameLane(_prevLane[i])) return true;

    return false;
}

bool lane::isConnected(const lane *l) const
{
    if (l == nullptr) return false;
    if (isSameLane(l)) return true;
    if (isNextLane(l)) return true;
    if (isPrevLane(l)) return true;
    return false;

}

bool lane::isConnectedButNotThis(const lane *l) const
{
    if (l == nullptr) return false;
    if (isNextLane(l)) return true;
    if (isPrevLane(l)) return true;
    return false;
}

bool lane::hasNextLane() const
{
    if (_nextLaneSize == 0) return false;
    return true;
}

bool lane::hasMultipleNextLanes() const
{
    if (_nextLaneSize > 1) return true;
    return false;
}

bool lane::hasPrevLane() const
{
    if (_prevLaneSize == 0) return false;
    return true;
}

bool lane::hasMultiplePrevLanes() const
{
    if (_prevLaneSize > 1) return true;
    return false;
}


void lane::setSpeed(const scalar speed)
{
    _speed = speed;
}

scalar lane::getSpeed() const
{
    return getSpeed(0);
    // return _speed;
}

scalar lane::getSpeed(scalar d) const
{
    scalar speed = _speed;
    for (uint i = 0; i < _odrSpeed.size(); ++i)
    {
        if (d  < _odrSpeed[i].s) break;
        speed = _odrSpeed[i].value;
    }
    return speed;
}


scalar lane::getWidth() const
{
    return _width;
}

scalar lane::getWidth(scalar d) const
{
    if (_constantWidth)
        return _width;

    scalar w = 0;
    int ndx = getGeometryIndex(d);
    if (ndx < 0) return 0;

    // Call so(s) and calculate it yourself:
    w = 0;
    scalar lo = 0;
    for (int i = 0; i < ndx; ++i )
        lo += _geom[i]->length();

    scalar t = _geom[ndx]->sl0(d - lo);
    if (!_odrFwd) t = maxSo() - t; // you still need to do that if you've flipped.
    for (uint i = 0; i < _odrWidth.size(); ++i)
    {
        if (!_odrWidth[i].inRange(t)) continue;
        scalar s = t - _odrWidth[i].s;
        scalar s2 = s * s;
        w += _odrWidth[i].a + _odrWidth[i].b * s + _odrWidth[i].c * s2 + _odrWidth[i].d * s * s2;
    }

    return w;
}

void lane::addTSign(tSign ts)
{
    std::cout << "[ Lane ] adding traffic sign: " << tSignInfoString(ts.info) << " to lane " << getSUID() << std::endl;

    // place the sign on the road:
    projectPointOntoLane(ts.lpos, ts.pos);
    //  and assign it an s coordinate.
    ts.s = unsafeDistanceFromTheBoL(ts.lpos);
    _tSigns.push_back(ts);
}


bool lane::hasTSigns() const
{
    if (_tSigns.size() > 0) return true;
    return false;
}

uint lane::tSignsSize() const
{
    return static_cast<uint>(_tSigns.size());
}

lane::tSignInfo lane::getTSignInfo(uint i) const
{
    return _tSigns[i].info;
}

scalar lane::getTSignSCoord(uint i) const
{
    return _tSigns[i].s;
}



void lane::addStaticObj(conflict::staticObj so)
{
    if (so.kind == conflict::staticObjKind::unknown) return;

    if (so.kind == conflict::staticObjKind::crosswalk)
        addCrosswalk(so);

    return;
}



bool lane::addCrosswalk(const conflict::staticObj &so)
{
    if (so.kind != conflict::staticObjKind::crosswalk)
    {
        std::cerr << "[ Error ] trying to configure a crosswalk with something else!" << std::endl;
        return false;
    }

    conflict cw;
    cw.s = so.s;
    cw.so = so.s - 0.5 * so.w;
    cw.se = so.s + 0.5 * so.w;
    cw.k = conflict::kind::zebraWalk;

    if (!getPointAtDistance(cw.pos, so.s))
    {
        std::cerr << "[ Error ] the object to be added is too far away!" << std::endl;
        return false;
    }

    scalar w = 0.5 * so.w;
    arr2 tmp;
    if (!getPointAtDistance(tmp, so.s - w))
        std::cerr << "[ Warning ] the object to be added is so big that it crosses to the previous lane!" << std::endl;

    if (!getPointAtDistance(tmp, so.s + w))
        std::cerr << "[ Warning ] the object to be added is so big that it crosses to the next lane!" << std::endl;

    addConflict(cw);

    return true;
}


void lane::addConflict(const conflict &cf)
{
    // Store in order:
    if (_conflicts.size() == 0) _conflicts.push_back(cf);
    else if (_conflicts.size() == 1)
    {
        _conflicts.resize(_conflicts.size() + 1);
        if (cf.s > _conflicts[0].s) _conflicts[1] = cf;
        else
        {
            _conflicts[1] = _conflicts[0];
            _conflicts[0] = cf;
        }
    }
    else
    {
        bool insertion = false;
        _conflicts.resize(_conflicts.size() + 1);
        for (int i = static_cast<int>(_conflicts.size()) - 2; i >= 0; --i)
        {
            if (_conflicts[i].s > cf.s)
                _conflicts[i+1] = _conflicts[i];

            else
            {
                _conflicts[i+1] = cf;
                insertion = true;
                break;
            }
        }
        if (!insertion) _conflicts[0] = cf;
    }

    // std::cout << " new conflict added to " << getCSUID() << std::endl << cf.print() << std::endl;
}

bool lane::hasConflicts() const
{
    if (_conflicts.size() > 0) return true;
    return false;
}

uint lane::conflictsSize() const
{
    return static_cast<uint>(_conflicts.size());
}

conflict lane::getConflict(uint i) const
{
    return _conflicts[i];
}

conflict lane::getConflict(scalar s) const
{
    return getConflict(static_cast<uint>(getConflictIdx(s)));
}

std::vector<conflict> lane::getConflicts() const
{
    return _conflicts;
}

arr2 lane::getConflictPos(uint i) const
{
    return _conflicts[i].pos;
}

arr2 lane::getConflictPos(scalar s) const
{
    int idx = getConflictIdx(s);
    if (idx < 0)
    {
        std::cout << "[ lane ] " << getSUID()
                  << " unable to return a valid position for conflict on " << s << std::endl;
        return {0, 0};
    }

    return _conflicts[idx].pos;
}

scalar lane::getConflictLength(uint i) const
{
    return _conflicts[i].se - _conflicts[i].so;
}

scalar lane::getConflictLength(scalar s) const
{
    return getConflictLength(static_cast<uint>(getConflictIdx(s)));
}

bool lane::addConflictLane(uint i, lane *l)
{
    if (i > _conflicts.size() -1)
    {
        std::cerr << "[ lane ] not enough crosswalks have been set for lane " << getSUID() << std::endl;
        return false;
    }

     // If the lane was already there, we're done.
    for (uint i = 0; i < _conflicts[i].hpLane.size(); ++i)
        if (l->isSameLane(_conflicts[i].hpLane[i])) return true;

    // Otherwise store it and return happiness:
    _conflicts[i].hpLane.push_back(l);
    return true;
}

bool lane::addConflictLane(scalar s, lane *l)
{
    int idx = getConflictIdx(s);
    if (idx < 0)
    {
        std::cout << "[ lane ] " << getSUID()
                  << " unable to add a lane for conflict on " << s << std::endl;
        return false;
    }

    return addConflictLane(static_cast<uint>(idx), l);
}

bool lane::conflictIsCrosswalk(uint i) const
{
    if (_conflicts[i].k == conflict::kind::zebraWalk) return true;
    return false;
}

bool lane::conflictIsMerge(uint i) const
{
    if (conflict::isMerge(_conflicts[i].k)) return true;
    return false;
}


/*
bool lane::conflictIsEoL(uint i) const
{
    if (mvf::areCloseEnough(_length, _conflicts[i].s, odrTol)) return true;
    return false;
}
*/

uint lane::crosswalksSize() const
{
    uint size = 0;
    for (uint i = 0; i < _conflicts.size(); ++i)
    {
        if (_conflicts[i].k == conflict::kind::zebraWalk) size += 1;
    }
    return size;
}

std::vector<const lane*> lane::getConflictLanes(uint i) const
{
    return _conflicts[i].hpLane;
}

std::vector<const lane*> lane::getConflictLanes(scalar s) const
{
    return getConflictLanes(static_cast<uint>(getConflictIdx(s)));
}


scalar lane::getConflictSCoord(uint i) const
{
    return _conflicts[i].s;
}

int lane::getConflictIdx(const lane *l) const
{
    for (size_t i = 0; i < _conflicts.size(); ++i)
    {
        for (size_t j = 0; j < _conflicts[i].hpLane.size(); ++j)
            if (l->isSameLane(_conflicts[i].hpLane[j])) return static_cast<int>(i);
    }


    return -1;
}


int lane::getConflictIdx(scalar s) const
{
    for (uint i = 0; i < _conflicts.size(); ++i)
    {
        if (mvf::areCloseEnough(_conflicts[i].s, s, 5e-2)) // 0.5*getConflictLength(i)))
            return static_cast<int>(i);
    }
    return -1;
}


std::vector<conflict::cuid> lane::getConflictLinks(uint i) const
{
    return _conflicts[i].links;
}

std::vector<conflict::cuid> lane::getConflictLinks(scalar s) const
{
    return getConflictLinks(static_cast<uint>(getConflictIdx(s)));
}

void lane::addConflictLink(uint i, conflict::cuid id)
{
    _conflicts[i].links.push_back(id);
}

conflict::kind lane::getConflictKind(uint i) const
{
    return _conflicts[i].k;
}


conflict::kind lane::getConflictKind(scalar s) const
{
    return getConflictKind(static_cast<uint>(getConflictIdx(s)));
}

void lane::setConflictKind(uint i, conflict::kind k)
{
    _conflicts[i].k = k;
}

void lane::setConflictKind(scalar s, conflict::kind k)
{
    return setConflictKind(static_cast<uint>(getConflictIdx(s)), k);
}

void lane::setConflictHPLanes(uint i, lane* hpLane, scalar anticipationTime)
{
    _conflicts[i].hpLane.clear();
    conflict::fillInHPLanes(_conflicts[i], hpLane, anticipationTime);
}

void lane::setConflictHPLanes(scalar s, lane *hpLane, scalar anticipationTime)
{
    return setConflictHPLanes(static_cast<uint>(getConflictIdx(s)), hpLane, anticipationTime);
}



std::vector<lane::tSign> lane::getTSigns() const
{
    return _tSigns;
}

lane::kind lane::getKind() const
{
    return _kind;
}

void lane::setKind(kind k)
{
    _kind = k;
}

bool lane::isRoundabout() const
{
    if (_kind == kind::roundabout) return true;
    return false;
}

bool lane::isCrosswalk() const
{
    if (_kind == kind::crosswalk) return true;
    return false;
}

bool lane::isPavement() const
{
    if (_kind == kind::pavement) return true;
    return false;
}

bool lane::isOdrTransitable(Odr::Kind::LaneType lt) const
{
    if ((lt == Odr::Kind::LaneType::driving) || (lt == Odr::Kind::LaneType::slipLane) ||
        (lt == Odr::Kind::LaneType::entry) || (lt == Odr::Kind::LaneType::exit) ||
        (lt == Odr::Kind::LaneType::onRamp) || (lt == Odr::Kind::LaneType::connectingRamp) ||
        (lt == Odr::Kind::LaneType::slipLane) )
        return true;

    return false;

}

bool lane::isOdrTransitable(const char* c) const
{
    Odr::Kind::LaneType lt = Odr::laneTypeFromCString(c);
    return isOdrTransitable(lt);
}

bool lane::isTransitable() const
{

    if ( (_kind == kind::tarmac) || (_kind == kind::roundabout) ||
         (_kind == kind::pavement) || (_kind == kind::crosswalk) )
        return true;

    return false;
}

bool lane::isToMerge() const
{
    for (uint i = 0; i < _conflicts.size(); ++i)
        if (conflict::isMerge(_conflicts[i].k)) return true;
    return false;
}

bool lane::isInOdrRange(scalar s) const
{
    return mvf::isInRangeLR(s, _odrSo, _geom.back()->roadSe());
}

void lane::setZero(const lane* z)
{
    _odrZero = z;
}

scalar lane::sli(scalar s0) const
{
    if (mvf::areCloseEnough(s0, 0., 1e-8)) return 0;

    if (!_odrZero)
    {
        std::cerr << "[ Error ] lane::sli can't calculate with _odrZero set to nullptr" << std::endl;
        return 0;
    }

    if (mvf::areCloseEnough(s0, _odrZero->getLength(), 1e-8))
        return _length;

    arr2 p0;
    if (!_odrZero->getPointAtDistance(p0, s0))
    {
        std::cerr << "[ Error ] lane::sli couldn't find a point at distance " << s0 << " for lane " << getCSUID() << std::endl;
        return 0;
    }
    arr2 pi = projectPointOntoLane(p0);
    return unsafeDistanceFromTheBoL(pi);
}

bool lane::actorsSupport(lane::kind k) const
{
    if (_kind == k) return true;

    if ( (_kind == kind::tarmac) && (k == kind::roundabout) ) return true;
    if ( (k == kind::tarmac) && (_kind == kind::roundabout) ) return true;

    if ( (_kind == kind::pavement) && (k == kind::crosswalk) ) return true;
    if ( (k == kind::pavement) && (_kind == kind::crosswalk) ) return true;

    return false;
}

bool lane::actorsSupport(concepts::actor k) const
{
    if ( (k == concepts::actor::car) &&
         ((_kind == kind::tarmac) || (_kind == kind::roundabout)) ) return true;
    if ( (k == concepts::actor::meeple) &&
         ((_kind == kind::pavement) || (_kind == kind::crosswalk)) ) return true;
    return false;
}

#ifdef QT_CORE_LIB
QPainterPath lane::getQPainterPath(uint n) const
{
    QPainterPath qpp;
    for (uint i = 0; i < _geom.size(); ++i)
        qpp += _geom[i]->getQPainterPath(n);

    return qpp;
}

std::vector<QPainterPath> lane::getQPainterPaths(uint n) const
{
    std::vector<QPainterPath> qpp;
    for (uint i = 0; i < _geom.size(); ++i)
        qpp.push_back(_geom[i]->getQPainterPath(n));

    return qpp;
}

std::vector<QPainterPath> lane::getBoxQPainterPaths() const
{
    std::vector<QPainterPath> qpp;
    for (uint i = 0; i < _geom.size(); ++i)
    {
        QPainterPath qpi;
        qpi.moveTo(ct::mToPix * _geom[i]->blc()[0], -ct::mToPix * _geom[i]->blc()[1]);
        qpi.lineTo(ct::mToPix * _geom[i]->trc()[0], -ct::mToPix * _geom[i]->blc()[1]);
        qpi.lineTo(ct::mToPix * _geom[i]->trc()[0], -ct::mToPix * _geom[i]->trc()[1]);
        qpi.lineTo(ct::mToPix * _geom[i]->blc()[0], -ct::mToPix * _geom[i]->trc()[1]);
        qpi.lineTo(ct::mToPix * _geom[i]->blc()[0], -ct::mToPix * _geom[i]->blc()[1]);
        qpp.push_back(qpi);
    }
    return qpp;
}


QPainterPath lane::getEdgeQPainterPath(uint n, int e) const
{
    QPainterPath qpp;
    if ((e != -1) && (e != 1))
    {
        std::cerr << "[ Error ] getEdgeQPainterPath means neither left nor right" << std::endl;
        return qpp;
    }

    if (n == 0) n = _length; // so that later appDs is 1m.
    scalar appDs = _length / n;

    scalar s = 0; // total distance down the road.
    bool first = true;
    for (uint i = 0; i < _geom.size(); ++i)
    {
        scalar si = 0;
        uint qi = std::floor(_geom[i]->length() / appDs);
        if (qi < 4) qi = 4;
        scalar dsi = _geom[i]->length() / qi;
        bool quit = false;
        for (uint j = 0; j <= qi; ++j)
        {
            arr2 ci;
            if (!_geom[i]->getPointAfterDistance(ci, _geom[i]->origin(), si))
            {
                ci = _geom[i]->dest();
                si = _geom[i]->length();
                quit = true;
            }
            arr2 ti = _geom[i]->getTangentInPoint(ci);
            arr2 ni;
            if (e == -1) ni = {-ti[1], ti[0]};
            else ni = {ti[1], -ti[0]};
            scalar w = getWidth(s + si);
            arr2 pi = {ci[0] + 0.48 * w * ni[0], ci[1] + 0.48 * w * ni[1]};
            if (first)
            {
                first = false;
                qpp.moveTo(QPointF(ct::mToPix * pi[0], -ct::mToPix * pi[1]));
            }
            else
                qpp.lineTo(QPointF(ct::mToPix * pi[0], -ct::mToPix * pi[1]));
            si += dsi;
            if (quit)
                break;
        }
        s += _geom[i]->length();
    }

    return qpp;
}

int lane::fillInVerticesAndIndices(scalar step, std::vector<QByteArray> &indexBytes, std::vector<QByteArray> &vertexBytes, std::vector<int> &indexSize, std::vector<int> &vertexSize) const
{
    if ((indexBytes.size() != 0) || (vertexBytes.size() != 0) ||
        (indexSize.size() != 0) || (vertexSize.size() != 0))
        return 1;

    vertexBytes.resize(_geom.size());
    indexBytes.resize(_geom.size());
    for (uint i = 0; i < _geom.size(); ++i)
    {
        int err = 0; //  _geom[i]->fillInVerticesAndIndices(step, indexBytes[i], vertexBytes[i], indexSize[i], vertexSize[i]);
        if (err > 0)
            return err;
    }

    return 0;


    /*
    // 1 - Preliminaries //
    // longitudinal //
    uint long_n = std::floor(_length / step);
    float long_d = _length / long_n;
    // lateral ///
    uint lat_n = std::floor(0.5 * _width / step);
    float lat_d = 0.5 * _width / lat_n;



    // 2 -
    vertexSize = (long_n+1) * (lat_n * 2 + 2); // numOfVertices
    vertexBytes.resize(3 * vertexSize * static_cast<int>(sizeof(float))); // bufferBytes
    float *positions = reinterpret_cast<float*>(vertexBytes.data());

    for (uint i = 0; i < long_n+1; ++i)
    {
        arr2 p_io = getPointAfterDistance(p)
        arr3 p_io = road.pointAfterDistance(long_d * i);
        arr3 l_io = {p_io[0] - 0.5f * road.width * road.no()[0],
                     p_io[1] - 0.5f * road.width * road.no()[1],
                     p_io[2] - 0.5f * road.width * road.no()[2]};

        for (uint j = 0; j < 2 * lat_n + 1; ++j)
        {
            *positions++ = static_cast<float>(l_io[0] + j * lat_d * road.no()[0]);
            *positions++ = static_cast<float>(l_io[1] + j * lat_d * road.no()[1]);
            *positions++ = static_cast<float>(l_io[2] + j * lat_d * road.no()[2]);
        }
    }

    */
}

#endif

std::vector<arr2> lane::getEdgePath(uint n, int edge) const
{
    std::vector<arr2> path;
    if ((edge != -1) && (edge != 1))
    {
        std::cerr << "[ Error ] getEdgePath means neither left nor right" << std::endl;
        return path;
    }

    if (n == 0) n = _length; // so that later appDs is 1m.
    scalar appDs = _length / n;

    scalar s = 0; // total distance down the road.
    bool first = true;
    for (uint i = 0; i < _geom.size(); ++i)
    {
        scalar si = 0;
        uint qi = std::floor(_geom[i]->length() / appDs);
        if (qi < 4) qi = 4;
        scalar dsi = _geom[i]->length() / qi;
        bool quit = false;
        for (uint j = 0; j <= qi; ++j)
        {
            arr2 ci;
            if (!_geom[i]->getPointAfterDistance(ci, _geom[i]->origin(), si))
            {
                ci = _geom[i]->dest();
                si = _geom[i]->length();
                quit = true;
            }
            arr2 ti = _geom[i]->getTangentInPoint(ci);
            arr2 ni;
            if (edge == -1) ni = {-ti[1], ti[0]};
            else ni = {ti[1], -ti[0]};
            scalar w = getWidth(s + si);
            arr2 pi = {ci[0] + 0.5 * w * ni[0], ci[1] + 0.5 * w * ni[1]};
            path.push_back(pi);
            si += dsi;
            if (quit)
                break;
        }
        s += _geom[i]->length();
    }

    return path;

}


// // CONFLICTS // //
// Static methods:
std::string conflict::kindString(conflict::kind k)
{
    switch(k)
    {
    case conflict::kind::free:
        return "free";
    case conflict::kind::giveWay:
        return "giveWay";
    case conflict::kind::stop:
        return "stop";
    case conflict::kind::mergePort:
        return "mergePort";
    case conflict::kind::mergeStarboard:
        return "mergeStarboard";
    case conflict::kind::zebraWalk:
        return "zebraWalk";
    case conflict::kind::unknown:
        return "unknown";
    default:
        return "Unrecognised conflict kind";
    }
}


bool conflict::areSameConflicts(const cuid &i, const cuid &j)
{
    if ( (mvf::areCloseEnough(i.s, j.s, 1e-2)) && (i.l == j.l)) return true;
    return false;
}

bool conflict::isMerge(conflict::kind k)
{
    if ((k == kind::mergePort) || (k == kind::mergeStarboard)) return true;
    return false;
}


scalar conflict::distance(const cuid &i, const cuid &j)
{
    if (i.l == j.l) return j.s - i.s;
    else if (i.l->isNextLane(j.l)) return j.s + i.l->getLength() - i.s;
    else if (i.l->isPrevLane(j.l)) return - (i.s + j.l->getLength() - j.s);
    else
    {
        std::cout << "[ Error ] conflict::distance cannot calculate the distance between too conflicts: "
                  << i.l->getSUID() << " in " << i.s << " and "
                  << j.l->getSUID() << " in " << j.s << " because the lanes are neither equal nor connected. Sorry. " << std::endl;
    }
    return 0;
}


bool conflict::is1stLinkedTo2nd(const cuid &ci, const cuid &cj)
{
    if ((ci.l == nullptr) || (cj.l == nullptr)) return false;
    if (areSameConflicts(ci, cj)) return true;
    std::vector<cuid> jLinks = cj.l->getConflictLinks(cj.s);
    for (uint j = 0; j < jLinks.size(); ++j)
    {
        if (areSameConflicts(jLinks[j], ci)) return true;
    }
    return false;



}

scalar conflict::calcStopMargin(const lane *l1, const lane *l2)
{
    scalar d = mvf::distance(l1->getDestination(), l2->getDestination());
    scalar s = l1->getLength();
    constexpr scalar ds = 0.1;
    constexpr scalar stdWidth = 3.5;
    scalar threshold = 1.0*0.5*((std::max)(stdWidth, l1->getWidth()) + (std::max)(stdWidth, l2->getWidth()));

    // std::cout << "threshold set to: " << threshold << " with l1.width: " << l1->getWidth() << " and l2.width: " << l2->getWidth() << std::endl;
    while (d < threshold)
    {
        s -= ds;
        arr2 p1;
        if (!l1->getPointAtDistance(p1, s))
        {
            std::cout << " calcStopMargin went too far... " << std::endl;
            s += ds;
            break;
        }
        arr2 p2;
        l2->projectPointOntoLane(p2, p1); // projectPointOntoLane only returns true.
        d = mvf::distance(p1, p2);
    }
    // std::cout << " stopMargin for " << l1->getSUID() << " should be set to: " << l1->getLength() - s << std::endl;
    return l1->getLength() - s;
}


conflict conflict::createEoLConflict(const lane* portLane, lane* hpLane, scalar anticipationTime)
{
    conflict cnf;
    cnf.pos = portLane->getDestination();
    cnf.s = portLane->getLength();
    cnf.so = cnf.s - conflict::calcStopMargin(portLane, hpLane);
    cnf.se = cnf.s + hpLane->getWidth();
    cnf.k = conflict::kind::giveWay;
    // scalar a =
    fillInHPLanes(cnf, hpLane, anticipationTime);
    // std::cout << "anticipation arrives to: " << a << " seconds" << std::endl;
    return cnf;
}


conflict conflict::createMergeConflict(const lane *accLane, mvf::side mergeSide)
{
    conflict cnf;
    cnf.pos = accLane->getDestination();
    cnf.s = accLane->getLength();
    cnf.so = cnf.s;
    cnf.se = cnf.s;
    if (mergeSide == mvf::side::port) cnf.k = kind::mergePort;
    else if (mergeSide == mvf::side::starboard) cnf.k = kind::mergeStarboard;
    else std::cout << "[ Error ] creating a merge conflict that is neither port nor starboard" << std::endl;
    return cnf;
}


conflict conflict::createIntersectionConflict(const arr2 &pos, const lane *lpLane, lane *hpLane, scalar anticipationTime)
{
    scalar factor = 1;
    if (lpLane->isCrosswalk()) factor = 0.5;

    conflict cnf;
    cnf.pos = pos;
    cnf.s = lpLane->unsafeDistanceFromTheBoL(pos);

    // let's check, because the end of the conflict is different, depending on the angle that the roads make:
    //   if they're:
    //    \           |              /
    //     \          |             /
    //      \         |            /
    //    --<--   ------<---    ------<--       the "vertical" ones always driving nortwards.
    //        \       ^          /
    //         \      |         /
    arr2 lpt, hpt;
    lpLane->getTangentInPoint(lpt, pos);
    hpLane->getTangentInPoint(hpt, pos);
    scalar angle = mvf::subtendedAngle(hpt, lpt);
    // std::cout << "conflict in " << pos[0] << ", " << pos[1] << " between lanes " << lpLane->getSUID()
    //           << " and lane " << hpLane->getSUID() << " has an angle of " << angle * ct::rad2deg << " degrees " << std::endl ;
    if (std::fabs(angle) > 1.92) factor = 1.5; // greater than 110 degrees.
    else if (std::fabs(angle) < 1.22) factor = 0.75;  // smaller than 70 degrees.


    cnf.so = cnf.s - factor * hpLane->getWidth();
    cnf.se = cnf.s + factor * hpLane->getWidth();
    cnf.k = conflict::kind::giveWay;
    fillInHPLanes(cnf, hpLane, anticipationTime);
    // scalar a = fillInHPLanes(cnf, hpLane, anticipationTime);
    // std::cout << "anticipation arrives to: " << a << " seconds" << std::endl;
    return cnf;
}


scalar conflict::fillInHPLanes(conflict &cnf, const lane *hpLane, scalar anticipationTime)
{
    cnf.hpLane.clear();
    cnf.hpLane.push_back(hpLane);
    scalar ant_i = hpLane->unsafeDistanceFromTheBoL(cnf.pos) / hpLane->getSpeed();
    const lane *l_i = hpLane;
    while (ant_i < anticipationTime) // there should be a while here:
    {
        scalar ant_j = 0;
        auto [ prevLane, prevLaneSize ] = l_i->getPrevLanes();

        for (uint pls = 0; pls < prevLaneSize; ++pls)
        {
            cnf.hpLane.push_back(prevLane[pls]);
            if (ant_j < prevLane[pls]->getLength() / prevLane[pls]->getSpeed())
                ant_j = prevLane[pls]->getLength() / prevLane[pls]->getSpeed();
        }
        ant_i += ant_j;

        if (prevLaneSize == 1) l_i = prevLane[0];
        else break;
    }
    // std::cout << "anticipation arrives to: " << ant_i << " seconds" << std::endl;
    return ant_i;
}

std::string conflict::print() const
{
    std::stringstream ss;
    ss << "Conflict of kind " << kindString(k) << " in s:" << s << ", so/se: (" << so << ", " << se << ") ";
    if (hpLane.size())
    {
        if (hpLane.size() == 1)
            ss << " with hpLane:" << std::endl;
        else
            ss << " with hpLanes:" << std::endl;
        for (uint i = 0; i < hpLane.size(); ++i)
            ss << " - " << hpLane[i]->getCSUID() << std::endl;
    }
    if (links.size())
    {
        ss << " with links in: " << std::endl;
        for (uint i = 0; i < links.size(); ++i)
            ss << links[i].l->getCSUID() << " s: " << links[i].s << std::endl;
    }
    return ss.str();
}

