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

#include "lane.h"
// DEBUG //
#include <fstream>
// #include <boost/format.hpp>

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

void lane::base()
{
    _width = 3;
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
    _length = 0;

    _shape = mvf::shape::unknown;

    _section = nullptr;
    _sectionID = -1;

    _sign = sign::o;

    _odrID = (std::numeric_limits<int>::max)();
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

void lane::set(const std::vector<Odr::geometry> &odrg, std::vector<Odr::offset> off, const Odr::smaL &odrL, scalar endingS)
{
    _odrID = odrL.odrID;
    _length = odrL.length; ///< that's a good estimate, but it's not the real length of the lane; just the length of the section at tis centre.

    lane::sign s = lane::sign::o;
    if (odrL.sign == -1) s = lane::sign::n;
    else if (odrL.sign == 1) s = lane::sign::p;

    initialise(odrL.w[0].a, odrL.speed, mvf::shape::opendrive, s);

    if (odrL.kind.compare(Odr::Kind::Driving) == 0)
        _kind = kind::tarmac;
    else if (odrL.kind.compare(Odr::Kind::Sidewalk) == 0)
        _kind = kind::pavement;
    else
        _kind = kind::unknown;

    bool geomPrint = false;

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
                _geom.push_back(new straight(odrg[i], getSignInt(), off[0].a, soi, sei));
            else if (odrg[i].g == Odr::Attr::Geometry::arc)
                _geom.push_back(new arc(odrg[i], getSignInt(), off[0].a, soi, sei));
            else if (odrg[i].g == Odr::Attr::Geometry::paramPoly3)
                _geom.push_back(new paramPoly3(odrg[i], getSignInt(), off[0].a, soi, sei));
            else if (odrg[i].g == Odr::Attr::Geometry::spiral)
                _geom.push_back(new vwSpiral(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
            else
            {
                std::cerr << "[ Lane ] Unsupported shape! The code will crash quickly after this." << std::endl;
                continue;
            }
        }
        else
        {
            if (odrg[i].g == Odr::Attr::Geometry::line)
                _geom.push_back(new vwStraight(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
            else if (odrg[i].g == Odr::Attr::Geometry::arc)
                _geom.push_back(new vwArc(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
            else if (odrg[i].g == Odr::Attr::Geometry::paramPoly3)
                _geom.push_back(new vwParamPoly3(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
            else if (odrg[i].g == Odr::Attr::Geometry::spiral)
                _geom.push_back(new vwSpiral(odrg[i], getSignInt(), off, soi, sei, roadSoi, geomPrint));
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

    scalar ds = numerical::defaultDs(_length);
    uint size = 1 + static_cast<uint>(std::round(_length / ds));
    numerical::initialise(ds, size);
    if (numerical::setup())
    {
        std::cout << "[ Error ] on " << getCSUID() << " in configuring numerical for the top class" << std::endl;
    }

    calcBoundingBox();


    if (geomPrint)
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

        /*
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
            if (_geom[i]->shape() == mvf::shape::vwStraight)
            {
                p = static_cast<vwStraight*>(_geom[i])->points();
                s = static_cast<vwStraight*>(_geom[i])->S();
            }
            else if (_geom[i]->shape() == mvf::shape::vwArc)
            {
                p = static_cast<vwArc*>(_geom[i])->points();
                s = static_cast<vwArc*>(_geom[i])->S();
            }
            else if (_geom[i]->shape() == mvf::shape::vwParamPoly3)
            {
                p = static_cast<vwParamPoly3*>(_geom[i])->points();
                s = static_cast<vwParamPoly3*>(_geom[i])->S();
            }
            else if (_geom[i]->shape() == mvf::shape::vwSpiral)
            {
                p = static_cast<vwSpiral*>(_geom[i])->points();
                s = static_cast<vwSpiral*>(_geom[i])->S();
            }

            for (uint j = 0; j < p.size(); ++j)
                f << boost::format("%12.3f %12.3f") %
                     p[j][0] % p[j][1] << std::endl;

            for (uint j = 0; j < s.size(); ++j)
                ff << boost::format("%12.3f") % s[j] << std::endl;
        }
        f.close();
        ff.close();
        */
    }

    return;

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
    default:
        return false;
    }
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
        lane** oldNextLane = _nextLane;
        _nextLaneSize = _prevLaneSize;
        _nextLane = _prevLane;

        _prevLaneSize = oldNextLaneSize;
        _prevLane = oldNextLane;

        lane* oldPortLane = _portLane;
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

    scalar ds = numerical::defaultDs(_length);
    uint size = 1 + static_cast<uint>(std::round(_length / ds));
    numerical::initialise(ds, size);
    numerical::setup();
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

    scalar ds = numerical::defaultDs(_length);
    uint size = 1 + static_cast<uint>(std::round(_length / ds));
    numerical::initialise(ds, size);
    numerical::setup();
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
    _width = t._width;
    _id = t._id;
    _length = t._length;
    _speed = t._speed;
    _isPermanent = t._isPermanent;
    _kind = t._kind;

    _bbblc = {t._bbblc[0], t._bbblc[1]};
    _bbtrc = {t._bbtrc[0], t._bbtrc[1]};

    if (t.hasNextLane())
    {
        _nextLaneSize = t._nextLaneSize;
        _nextLane = new lane*[_nextLaneSize];
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
        _prevLane = new lane*[_prevLaneSize];
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

        else
            std::cout << "[ WARNING ] lane::assignInputToThis doesn't know about this geometry" << std::endl;
    }
    _odrWidth = t._odrWidth;


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



void lane::setPrevLane(lane *l, bool crosslink)
{
    if (isPrevLane(l)) return;

    if (_prevLaneSize == 0)
        _prevLane = new lane*[1];
    else
    {
        // check that l was previously not there:
        for (uint i = 0; i < _prevLaneSize; ++i)
            if (_prevLane[i]->isSameLane(l)) return;

        // now carry on with the assignment:
        lane **tmp = new lane*[_prevLaneSize];
        for (uint i = 0; i < _prevLaneSize; ++i)
            tmp[i] = _prevLane[i];
        delete[] _prevLane;
        _prevLane = new lane*[_prevLaneSize + 1];
        for (uint i = 0; i < _prevLaneSize; ++i)
            _prevLane[i] = tmp[i];
        delete[] tmp;
    }
    _prevLane[_prevLaneSize] = l;
    _prevLaneSize += 1;

    // Curved lanes cannot be configured without a previous lane:
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

    // Now add this lane as a next lane of l:
    else if ( (_isPermanent) && ( (!isCrosswalk())) && (crosslink) )
         l->setNextLane(this);
}


void lane::setNextLane(lane *l, bool crosslink)
{
    if (isNextLane(l)) return;

    if (_nextLaneSize == 0)
       _nextLane = new lane*[1];
    else
    {
        // check that l was previously not there:
        for (uint i = 0; i < _nextLaneSize; ++i)
            if (_nextLane[i]->isSameLane(l)) return;

        // now carry on with the assignment:
       lane **tmp = new lane*[_nextLaneSize];
       for (uint i = 0; i < _nextLaneSize; ++i)
         tmp[i] = _nextLane[i];
       delete[] _nextLane;
       _nextLane = new lane*[_nextLaneSize + 1];
       for (uint i = 0; i < _nextLaneSize; ++i)
         _nextLane[i] = tmp[i];
       delete[] tmp;
    }
    _nextLane[_nextLaneSize] = l;
    _nextLaneSize += 1;

    // Now add this lane as a previous lane of l:
    if ((_isPermanent) && ((!isCrosswalk()) && (crosslink)))
        l->setPrevLane(this);
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

void lane::setPortLane(lane *l)
{
    _portLane = l;
}

void lane::setStarboardLane(lane *l)
{
    _starboardLane = l;
}


void lane::setSection(section &s)
{
    _section = &s;
}


lane* lane::getNextLane() const
{
    return getNextLane(0);
}

lane* lane::getNextLane(uint idx) const
{
    return _nextLane[idx];
}

std::tuple<lane**, size_t> lane::getNextLanes() const
{
    return std::make_tuple(_nextLane, _nextLaneSize);
}

size_t lane::getNextLaneSize() const
{
    return _nextLaneSize;
}

lane* lane::getPrevLane() const
{
    return getPrevLane(0);
}

lane* lane::getPrevLane(uint idx) const
{
    return _prevLane[idx];
}

std::tuple<lane**, size_t> lane::getPrevLanes() const
{
    return std::make_tuple(_prevLane, _prevLaneSize);
}

size_t lane::getPrevLaneSize() const
{
    return _prevLaneSize;
}

lane* lane::getPortLane() const
{
    return _portLane;
}

lane* lane::getStarboardLane() const
{
    return _starboardLane;
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

lane* lane::getMergeLane() const
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
        int lineIdx = getGeomIndexForPoint(p);
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
        else
            std::cout << "[ WARNING ] lane::getGeometries() - Unknown shape" << std::endl;


    }
    return v;

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

int lane::getOdrID() const
{
    return _odrID;
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
    return getSUID();
}

std::string lane::getOdrSUID() const
{
    if (!isOpenDrive()) return "";
    return sUID(_odrSectionID, _odrID);
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


void lane::getTangentInPoint(arr2 &t, const arr2 &p) const
{
    if (_geom.size() == 1)
        t = _geom[0]->getTangentInPoint(p);
    else if (_geom.size() > 1)
    {
        int lineIdx = getGeomIndexForPoint(p);
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


int lane::getGeomIndexForPoint(const arr2 &p) const
{
    bool fast = true;
    if (fast) /* Fast and slightly insecure */
    {
        for (uint i = 0; i < _geom.size(); ++i)
            if (mvf::isPointInBoxBLcTRcTol(p, _geom[i]->blc(), _geom[i]->trc(), odrTol))
                return i;
        return -1;
    }


    /* Slow but safe */
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
}


bool lane::getPointAfterDistance(arr2 &p, const arr2 &o, scalar d) const
{
    if (_geom.size() == 1)
    {
        if (!_geom[0]->getPointAfterDistance(p, o, d)) return false;
    }
    else if (_geom.size() > 1)
    {
        int lineIdx = getGeomIndexForPoint(o);
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
        int lineIdx = getGeomIndexForPoint(p);
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
void lane::getPointWithOffset(arr2 &p, arr2 &o, scalar loff) const
{
    arr2 v = getTangentInPoint(o);
    scalar angle = 0.5 * ct::pi;
    mvf::rotateVectorByAngle(v, angle);
    p = {o[0] + loff * v[0], o[1] + loff * v[1]};
    return;
}

void lane::getPointWithOffset(arr2 &p, scalar s, scalar loff) const
{
    arr2 o;
    getPointAtDistance(o, s);
    getPointWithOffset(p, o, loff);
    return;
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
                _geom[i]->getPointAfterDistance(p, _geom[i]->origin(), s);

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
        _pointsSize = ndx + 1;
        std::cout << "[ WARNING! ] we changed _pointsSize: " << _pointsSize << " to " << ndx + 1 << std::endl;
    }

    _pointsX[_pointsSize -1] = _geom.back()->dest()[0]; // _dest[0];
    _pointsY[_pointsSize -1] = _geom.back()->dest()[1]; // _dest[1];

}

std::vector<arr2> lane::getIntersectionPoints(const lane *l) const
{
    // In this case we know it is odr vs odr, and thus we will do numerically with the following approach
    // 1 - Check whether the two bounding boxes do overlap;
    // 2 - if not, return no points;
    // 3 - If overlapping split this into this.1 and this.2 and l into l.1 and l2.
    // 4 - Check the overlapping between the pairs this vs l
    // 5 - Keep the overlapping boxes, and keep halving the curves until every intersection has been found.

    std::vector<arr2> intersections;

    if ((_pointsSize == 0) || (l->_pointsSize == 0)) return intersections;

    if (!mvf::boxesOverlap(_bbblc, _bbtrc, l->_bbblc, l->_bbtrc)) return intersections;
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


bool lane::projectPointOntoLane(arr2 &p, const arr2 &o, bool verbose) const
{
    if (_geom.size() == 1)
    {
        arr2 q = o;
        p = _geom[0]->projectPointHere(q);
    }
    else if (_geom.size() > 1)
    {
        scalar d2 = 1e12;
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
    return _speed;
}

scalar lane::getWidth() const
{
    return _width;
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


void lane::crosslinkConflict(uint cndx, conflict::uid ocuid)
{
    conflict::uid tcuid = {this, _conflicts[cndx].s};
    if (!conflict::is1stLinkedTo2nd({ocuid.l, ocuid.s}, {tcuid.l, tcuid.s}))
        _conflicts[cndx].links.push_back(ocuid);

    if (!conflict::is1stLinkedTo2nd({tcuid.l, tcuid.s}, {ocuid.l, ocuid.s}))
    {
        uint cuidj = ocuid.l->getConflictIdx(ocuid.s);
        ocuid.l->_conflicts[cuidj].links.push_back(tcuid); // {this, _conflicts[cndx].s});
    }


    std::vector<conflict::uid> tlinks = _conflicts[cndx].links ;
    for (uint t = 0; t < tlinks.size(); ++t)
    {
        if (!conflict::is1stLinkedTo2nd({ocuid.l, ocuid.s}, {tlinks[t].l, tlinks[t].s}))
        {
            uint ndx = tlinks[t].l->getConflictIdx(tlinks[t].s);
            tlinks[t].l->_conflicts[ndx].links.push_back(ocuid);
        }
        if (!conflict::is1stLinkedTo2nd({tlinks[t].l, tlinks[t].s}, {ocuid.l, ocuid.s}))
        {
            uint ndx = ocuid.l->getConflictIdx(ocuid.s);
            ocuid.l->_conflicts[ndx].links.push_back(tlinks[t]);
        }
    }

    std::vector<conflict::uid> olinks = ocuid.l->getConflictLinks(ocuid.s);
    for (uint o = 0; o < olinks.size(); ++o)
    {
        if (!conflict::is1stLinkedTo2nd({tcuid.l, tcuid.s}, {olinks[o].l, olinks[o].s}))
        {
            uint ndx = olinks[o].l->getConflictIdx(olinks[o].s);
            olinks[o].l->_conflicts[ndx].links.push_back(tcuid);
        }
        if (!conflict::is1stLinkedTo2nd({olinks[o].l, olinks[o].s}, {tcuid.l, tcuid.s}))
        {
            uint ndx = tcuid.l->getConflictIdx(tcuid.s);
            tcuid.l->_conflicts[ndx].links.push_back(olinks[o]);
        }
    }
}

void lane::crosslinkConflict(scalar cSCoord, conflict::uid cuid)
{
    int idx = getConflictIdx(cSCoord);
    if (idx < 0)
    {
        std::cout << "unable to find a conflict in lane " << getSUID() << " at length " << cSCoord << std::endl;
        return;
    }
    return crosslinkConflict(static_cast<uint>(idx), cuid);
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

std::vector<lane*> lane::getConflictLanes(uint i) const
{
    return _conflicts[i].hpLane;
}

std::vector<lane*> lane::getConflictLanes(scalar s) const
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


std::vector<conflict::uid> lane::getConflictLinks(uint i) const
{
    return _conflicts[i].links;
}

std::vector<conflict::uid> lane::getConflictLinks(scalar s) const
{
    return getConflictLinks(static_cast<uint>(getConflictIdx(s)));
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


bool lane::swapConflictPriority(uint ci)
{
    bool swapped = false;
    conflict::kind ko = _conflicts[ci].k;
    for (uint lj = 0; lj < _conflicts[ci].hpLane.size(); ++lj)
    {
        int ick = _conflicts[ci].hpLane[lj]->getConflictIdx(this);
        if (ick < 0) continue;
        uint uck = static_cast<uint>(ick);
        _conflicts[ci].k = _conflicts[ci].hpLane[lj]->getConflictKind(uck);
        _conflicts[ci].hpLane[lj]->setConflictKind(uck, ko);
        std::cout << "swapped priorities and now: " << getSUID() << ":" << conflict::kindString(_conflicts[ci].k)
                  << " and " << _conflicts[ci].hpLane[lj]->getSUID()
                  << ":" << conflict::kindString(_conflicts[ci].hpLane[lj]->getConflictKind(uck)) << std::endl;
        swapped = true;
    }
    return swapped;

}


bool lane::swapConflictPriority(scalar s)
{
    return swapConflictPriority(static_cast<uint>(getConflictIdx(s)));
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

int lane::fillInVerticesAndIndices(QByteArray &indexBytes, QByteArray &vertexBytes, int &indexSize, int &vertexSize) const
{
    return 0;
}

#endif


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

bool conflict::areSameConflicts(const uid &i, const uid &j)
{
    if ( (mvf::areCloseEnough(i.s, j.s, 1e-2)) && (i.l == j.l)) return true;
    return false;
}

bool conflict::areSameConflicts(const cuid &i, const cuid &j)
{
    if ( (mvf::areCloseEnough(i.s, j.s, 1e-2)) && (i.l == j.l)) return true;
    return false;
}

bool conflict::areSameConflicts(const uid &i, const cuid &j)
{
    if ( (mvf::areCloseEnough(i.s, j.s, 1e-2)) && (i.l == j.l)) return true;
    return false;
}

bool conflict::areSameConflicts(const cuid &i, const uid &j)
{
    if ( (mvf::areCloseEnough(i.s, j.s, 1e-2)) && (i.l == j.l)) return true;
    return false;
}

bool conflict::isMerge(conflict::kind k)
{
    if ((k == kind::mergePort) || (k == kind::mergeStarboard)) return true;
    return false;
}


scalar conflict::distance(const uid &i, const uid &j)
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
    std::vector<uid> jLinks = cj.l->getConflictLinks(cj.s);
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
        l2->projectPointOntoLane(p2, p1, false); // projectPointOntoLane only returns true.
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


scalar conflict::fillInHPLanes(conflict &cnf, lane *hpLane, scalar anticipationTime)
{
    cnf.hpLane.clear();
    cnf.hpLane.push_back(hpLane);
    scalar ant_i = hpLane->unsafeDistanceFromTheBoL(cnf.pos) / hpLane->getSpeed();
    lane *l_i = hpLane;
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
