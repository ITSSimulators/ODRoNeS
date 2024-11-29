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

#include "section.h"
using namespace odrones;

section::section() :
   _id(-1),
   _lanes(nullptr),
    _writtenSize(0),
   _allocSize(0)
{
    _bbblc = {0, 0};
    _bbtrc = {0, 0};
    _odrID = 0;
}

void section::set(size_t size)
{
    _allocSize = size;
    _lanes = new lane[size]();
    for (uint i = 0; i < size; ++i)
    {
        _lanes[i].setID(i);
        _lanes[i].setSectionID(_id);
        _lanes[i].setSection(*this);
    }
    _zero.setSection(*this);
    _zero.setSectionID(_id);
}


section::~section()
{
    if (_allocSize > 0) delete[] _lanes;
    _zero.clearMemory();
}


section::section(size_t size) :
    _writtenSize(0),
    _allocSize(size)
{
    set(size);
}


section::section(const section& s)
{
    assignInputSectionToThis(s);
}

section& section::operator=(const section &s)
{
    if (_allocSize > 0) delete[] _lanes;
    assignInputSectionToThis(s);
    return *this;
}

void section::assignInputSectionToThis(const section &s)
{

    _id = s.getID();
    _allocSize = s.getMaxSize();
    _writtenSize = s.size();
    if (_writtenSize > 0)
    {
        _lanes = new lane[_allocSize];
        for (size_t i = 0; i < _writtenSize; ++i)
            _lanes[i] = s._lanes[i];
    }
    else _lanes = nullptr;

    _zero = s._zero;

    _bbblc = {s._bbblc[0], s._bbblc[1]};
    _bbtrc = {s._bbtrc[0], s._bbtrc[1]};

    _odrID = s._odrID;
}


bool section::isReady()
{
    if (_allocSize == 0) return false;
    return true;
}


void section::updateBoundingBox(uint ndx)
{
    if (mvf::areSamePoints(_bbblc, _bbtrc))
        _lanes[ndx].getBoundingBox(_bbblc, _bbtrc);
    else
    {
        arr2 bli, tri;
        _lanes[ndx].getBoundingBox(bli, tri);
        mvf::increaseBoxWithBox(_bbblc, _bbtrc, bli, tri);
    }
}

void section::updateBoundingBox(const arr2 &blc, const arr2 &trc)
{
    if (mvf::areSamePoints(_bbblc, _bbtrc))
    {
        _bbblc = {blc[0], blc[1]};
        _bbtrc = {trc[0], trc[1]};
    }
    else
        mvf::increaseBoxWithBox(_bbblc, _bbtrc, blc, trc);
}

void section::addUpBoundingBoxes()
{

    if (!size()) return;
    _lanes[0].getBoundingBox(_bbblc, _bbtrc);
    for (uint i = 1; i < size(); ++i)
        updateBoundingBox(i);
}

void section::getBoundingBox(arr2 &blc, arr2 &trc) const
{
    blc = {_bbblc[0], _bbblc[1]};
    trc = {_bbtrc[0], _bbtrc[1]};
}


int section::addLane(const arr2 &origin, const arr2 &dest, scalar width, scalar speed, mvf::shape shp, lane::sign sgn)
{
    int err = 0;
    if (_writtenSize == _allocSize)
    {
        std::cerr << "cannot add more lanes to section " << _id << std::endl;
        err = 1;
    }
    else
    {
        _lanes[_writtenSize].setID(static_cast<int>(_writtenSize));
        _lanes[_writtenSize].set(origin, dest, width, speed, shp, sgn);
        updateBoundingBox(static_cast<uint>(_writtenSize));
        _writtenSize += 1;
    }
    return err;
}

int section::addLane(const arr2 &origin, const arr2 &dest, const arr2 &centre, scalar width, scalar speed, mvf::shape shp, lane::sign sgn, bool permanent)
{
    int err = 0;
    if (_writtenSize == _allocSize)
    {
        std::cerr << "cannot add more lanes to section " << _id << std::endl;
        err = 1;
    }
    else
    {
        _lanes[_writtenSize].setID(static_cast<int>(_writtenSize));
        _lanes[_writtenSize].set(origin, dest, centre, width, speed, shp, sgn, permanent);
        updateBoundingBox(static_cast<uint>(_writtenSize));
        _writtenSize += 1;
    }
    return err;

}

int section::addLane(const std::vector<bezier2> &bzr, scalar width, scalar speed, lane::sign sgn)
{
    int err = 0;
    if (_writtenSize == _allocSize)
    {
        std::cerr << "cannot add more lanes to section " << _id << std::endl;
        err = 1;
    }
    else
    {
        _lanes[_writtenSize].setID(static_cast<int>(_writtenSize));
        _lanes[_writtenSize].set(bzr, width, speed, sgn);
        updateBoundingBox(static_cast<uint>(_writtenSize));
        _writtenSize += 1;
    }
    return err;
}

int section::addLane(const std::vector<bezier3> &bzr, scalar width, scalar speed, lane::sign sgn)
{
    int err = 0;
    if (_writtenSize == _allocSize)
    {
        std::cerr << "cannot add more lanes to section " << _id << std::endl;
        err = 1;
    }
    else
    {
        _lanes[_writtenSize].setID(static_cast<int>(_writtenSize));
        _lanes[_writtenSize].set(bzr, width, speed, sgn);
        updateBoundingBox(static_cast<uint>(_writtenSize));
        _writtenSize += 1;
    }
    return err;
}


int section::addLane(const std::vector<Odr::geometry> &geom, const std::vector<Odr::offset> &off, const std::vector<Odr::offset> &width, const Odr::smaL &odrL, scalar se)
{
    int err = 0;
    if (_writtenSize == _allocSize)
    {
        std::cerr << "cannot add more lanes to section " << _id << std::endl;
        err = 1;
    }
    else
    {
        _lanes[_writtenSize].setID(static_cast<int>(_writtenSize));
        _lanes[_writtenSize].setOdrSectionID(_odrID);
        _lanes[_writtenSize].set(geom, off, width, odrL, se);
        updateBoundingBox(static_cast<uint>(_writtenSize));
        _writtenSize += 1;
    }
    return err;
}


int section::addLane(const OneVersion::smaS &sec, uint index)
{
    int err = 0;
    if (_writtenSize == _allocSize)
    {
        std::cerr << "cannot add more lanes to section " << _id << std::endl;
        err = 1;
    }
    else
    {
        _lanes[_writtenSize].setID(static_cast<int>(_writtenSize));
        _lanes[_writtenSize].set(sec, index);
        updateBoundingBox(static_cast<uint>(_writtenSize));
        _writtenSize += 1;
    }
    return err;
}




bool section::setIthConflictLane(uint ithCW, lane *cwl)
{
    for (uint li = 0; li < size(); ++li)
    {
        if (!_lanes[li].addConflictLane(ithCW, cwl))
            return false;
    }
    return true;
}

void section::setOneVersionRoad(const OneVersion::smaS &sec, uint lgID)
{
    _ovID = sec.ovID;
    _ovID.lgIndex = lgID;

    for (uint i = 0; i < sec.lanes.size(); ++i)
    {
        if (sec.lanes[i].ovID.lgIndex != lgID)
            continue;

        addLane(sec, i);
    }
}

void section::setOdrRoad(const Odr::smaS &sec, uint lsID)
{
    _odrID = sec.odrID;

    // Lanes within the same laneSection start at the same so and end at the same se,
    //  so start finding out these values:
    scalar so = 0;
    for (uint i = 0; i < sec.lanes.size(); ++i)
    {
        if (sec.lanes[i].ndxLS == lsID)
        {
            so = sec.lanes[i].startingS;
            break;
        }
    }
    scalar se = sec.lanes[0].length;
    for (uint i = 0; i < sec.lanes.size(); ++i)
    {
        if (sec.lanes[i].ndxLS == lsID + 1)
        {
            se = sec.lanes[i].startingS;
            break;
        }
    }
    // std::cout << "sec: " << sec.odrID << " starts in " << so << " and finishes in " << se << std::endl;


    // Now, for every lane, get an array with all the offsets,
    //   simplify it, and create the lane
    int validLane = -1; // we'll use that later to help out with a .
    for (uint i = 0; i < sec.lanes.size(); ++i)
    {
        if (sec.lanes[i].ndxLS != lsID) continue; // load only the lanes in this laneSection

        int odrSgn_i = sec.lanes[i].sign;
        // The offset of the laneSection:
        std::vector<Odr::offset> w_ls_i;
        for (uint j = 0; j < sec.loffset.size(); ++j)
        {
            Odr::offset off_j = sec.loffset[j];
            if (off_j.isNull()) continue; // if ( Odr::isOffsetNull(off_j) ) continue;
            if ( (w_ls_i.size()) && ( w_ls_i.back() == off_j) )   continue; // discard repeated loffsets
            if (!mvf::isInRangeL(off_j.s, so, se)) continue; // get only the loffsets in this region:
            w_ls_i.push_back(static_cast<scalar>(odrSgn_i) * off_j);
        }


        // Push back its own width:
        std::vector<Odr::offset> w_self_i;
        for (uint j = 0; j < sec.lanes[i].w.size(); ++j)
        {
            Odr::offset wj = sec.lanes[i].w[j];
            if ((!mvf::isInRangeL(wj.s + so, so, se)) || (wj.isNull())) // (Odr::isOffsetNull(wj)))
                continue;

            w_self_i.push_back(0.5 * wj);
            w_self_i.back().s += so;
        }

        // Important: the Standard states that lanes can't have both "width" and "border", and if anything it's width.
        // Odr::offset bi = sec.lanes[i].border;
        bool w_border_i = false;
        if (!w_self_i.size())
        {
            for (uint j = 0; j < sec.lanes[i].border.size(); ++j)
            {
                Odr::offset bj = sec.lanes[i].border[j];
                if ((!mvf::isInRangeL(bj.s + so, so, se)) || (bj.isNull()))
                    continue;
                w_self_i.push_back(0.5 * odrSgn_i * bj);
                w_self_i.back().s += so;
                w_border_i = true;
            }
        }


        // The offset of the lanes that are closer to the centre:
        int odrID_i = sec.lanes[i].odrID;
        std::vector<std::pair<int, Odr::offset>> w_prev_i;
        for (uint j = 0; j < sec.lanes.size(); ++j)
        {
            if (sec.lanes[i].ndxLS != sec.lanes[j].ndxLS) continue;
            if (odrSgn_i != sec.lanes[j].sign) continue;
            if (std::abs(odrID_i) <= std::abs(sec.lanes[j].odrID)) continue; // if (i == j) continue;
            bool wj = false;
            for (uint k = 0; k < sec.lanes[j].w.size(); ++k)
            {
                if ((!mvf::isInRangeL(sec.lanes[j].w[k].s + so, so, se)) ||
                        (sec.lanes[j].w[k].isNull())) continue;

                wj = true;
                w_prev_i.push_back(std::pair(std::abs(sec.lanes[j].odrID), sec.lanes[j].w[k]));
                w_prev_i.back().second.s += so;
            }
            // Important: the Standard states that lanes can't have both "width" and "border".
            if (wj) continue;
            for (uint k = 0; k < sec.lanes[j].border.size(); ++k)
            {
                if ((!mvf::isInRangeL(sec.lanes[j].border[k].s + so, so, se)) ||
                        (sec.lanes[j].border[k].isNull())) continue;
                w_prev_i.push_back(std::pair(std::abs(sec.lanes[j].odrID), sec.lanes[j].border[k]));
                w_prev_i.back().second.s += so;
            }
        }


        // Before anything else, we sort w_prev_i, lanes from the centre to the border
        //    and considering increasing s when the lane is the same;
        //    the other two, w_ls_i and w_self_i must be in order already.
        if (w_prev_i.size()) std::sort(w_prev_i.begin(), w_prev_i.end());

        // On a new array, put w_ls, w_prev, and w_sef, so that
        //    you "append" w_self using a "first" that is io + 1
        //    and "prepend"  w_ls using a "first" that is io -1;
        std::vector<std::pair<int, Odr::offset>> w_all_i;
        int first = 0;
        if (w_prev_i.size()) first = w_prev_i[0].first - 1;
        for (uint j = 0; j < w_ls_i.size(); ++j)
            w_all_i.push_back(std::pair(first, w_ls_i[j]));

        if (!w_border_i)
            w_all_i.insert(w_all_i.end(), w_prev_i.begin(), w_prev_i.end());

        // Finally, self:
        int last = 0;
        if (w_all_i.size()) last = w_all_i.back().first + 1;
        int width_first_ndx = w_all_i.size(); // remember that from here to the end its (self) width
        for (uint j = 0; j < w_self_i.size(); ++j)
            w_all_i.push_back(std::pair(last, w_self_i[j]));


        // Then calculate the end of each offset:
        for (uint j = 1; j < w_all_i.size(); ++j)
        {
            if (w_all_i[j].first == w_all_i[j-1].first)
                w_all_i[j-1].second.se = w_all_i[j].second.s;
            else
                w_all_i[j-1].second.se = se;
        }
        if (w_all_i.size())
            w_all_i.back().second.se = se;


        // And tell offset it's been set:
        for (uint j = 0; j < w_all_i.size(); ++j)
            w_all_i[j].second.seSet = true;


        // Finally, make a single array,
        //   discard those for which so = se and simplify:
        std::vector<Odr::offset> off;
        for (uint j = 0; j < w_all_i.size(); ++j)
        {
            if (mvf::areSameValues(w_all_i[j].second.s, w_all_i[j].second.se))
                continue;
            off.push_back(w_all_i[j].second);
        }

        std::vector<Odr::offset> s_off = Odr::offset::simplify(off);

        // And have the full sized width / border in a single separate vector:
        std::vector<Odr::offset> width;
        for (uint j = width_first_ndx; j < w_all_i.size(); ++j)
            width.push_back(2 * w_all_i[j].second);

        /* Print out offset details
        std::cout << "lane: " << sec.odrID << ":" << sec.lanes[i].odrID << ":" << lsID
                  << " has offset: " << std::endl;
        for (uint j = 0; j < off_i.size(); ++j)
        {
            std::cout << "[" << j << "] s: " << off_i[j].s << ", {" << off_i[j].a << ", "
                      << off_i[j].b << ", " << off_i[j].c << ", " << off_i[j].d << "}" << std::endl;
        }
        std::cout << " after being simplified from: " << std::endl;
        for (uint j = 0; j < s_off[i].size(); ++j)
        {
            std::cout << "[" << j << "] s: " << s_off[i][j].s << ", {" << s_off[i][j].a << ", "
                      << s_off[i][j].b << ", " << s_off[i][j].c << ", " << s_off[i][j].d << "}" << std::endl;
        }
        */

        // if (addLane(sec.geom, off_i, sec.lanes[i], se)) return;
        if (addLane(sec.geom, s_off, width, sec.lanes[i], se)) return;
        validLane = i;

        // and calculate the total bounding box for the section.
        if (_writtenSize == 1)
            _lanes[_writtenSize - 1].getBoundingBox(_bbblc, _bbtrc);
        else
        {
            arr2 blc_i, trc_i;
            _lanes[_writtenSize - 1].getBoundingBox(blc_i, trc_i);
            mvf::increaseBoxWithBox(_bbblc, _bbtrc, blc_i, trc_i);
        }
    }

    // Now we need to configure a laneZero in _zero if it has not happened yet:
    if ((_zero.getSign() == lane::sign::o) && (_zero.getKind() != lane::kind::none))
        setZero(sec.geom, so, se);

    // Now use getPointWithOffset(p, d, offset) to get the xy
    //   and calculate st's/
    for (uint i = 0; i < sec.tsigns.size(); ++i)
    {
        if (!mvf::isInRangeLR(sec.tsigns[i].s, so, se)) continue;

        lane::tSign lts;
        _zero.getPointWithOffset(lts.pos, static_cast<scalar>(sec.tsigns[i].s - so),
                                        static_cast<scalar>(sec.tsigns[i].t));
        lts.section = getID();

        if (sec.tsigns[i].name == "Sign_Yield")
            lts.info = lane::tSignInfo::giveWay;
        else if (sec.tsigns[i].name == "Sign_Stop")
            lts.info = lane::tSignInfo::stop;
        else
            std::cerr << "[ Error ] Unrecognised traffic sign name: " << sec.tsigns[i].name << std::endl;

        for (uint j = 0; j < _writtenSize; ++j)
        {
            if (sec.tsigns[i].orientation * _lanes[j].odrID() < 0) continue; // orientation may be -1, 0 or +1

            if (_lanes[j].odrID() > 0) lts.mDir = 1;
            else lts.mDir = -1;
            lts.lane = j;
            lts.assigned = true;
            _lanes[j].addTSign(lts);
        }
    }

    return;
}

bool section::setZero(const std::vector<Odr::geometry> &g, scalar so, scalar se)
{
    if ((_zero.getSign() != lane::sign::o) && (_zero.getKind() != lane::kind::unknown))
        return false;

    _zero.setID(-1); ///< I hope this doesn't cause havoc.
    _zero.setOdrSectionID(_odrID);

    Odr::smaL odrl0;
    odrl0.startingS = so;
    odrl0.sign = 1;
    odrl0.speed = 0;
    odrl0.odrID = 0;
    odrl0.kind = Odr::Kind::None;
    _zero.set(g, {Odr::offset(0.,0.,0.,0.,so,se)},
              {Odr::offset(0.,0.,0.,0.,so,se)}, odrl0, se);

    return true;
}


scalar section::maxSpeed() const
{
    scalar speed = 0;
    for (uint i = 0; i < _writtenSize; ++i)
        if (_lanes[i].getSpeed() > speed) speed = _lanes[i].getSpeed();

    return speed;
}


std::vector<lane::tSign> section::getTSigns() const
{
    std::vector<lane::tSign> ts_o;
    for (uint i = 0; i < _writtenSize; ++i)
    {
        std::vector<lane::tSign> ts_i = _lanes[i].getTSigns();
        for (uint j = 0; j < ts_i.size(); ++j)
        {
            bool missing = true;
            for (uint k = 0; k < ts_o.size(); ++k)
            {
                if ((mvf::areSamePoints(ts_i[j].pos, ts_o[k].pos)) &&
                    (ts_i[j].info == ts_o[k].info))
                {
                    missing = false;
                    break;
                }
            }
            if (missing) ts_o.push_back(ts_i[j]);
        }
    }

    return ts_o;

}


uint section::odrID() const
{
    return _odrID;
}

OneVersion::OVID section::ovID() const
{
    return _ovID;
}

void section::setOdrID(uint id)
{
    _odrID = id;
}

void section::setOVID(OneVersion::OVID id)
{
    _ovID = id;
}



bool section::flipBackwards()
{
    bool success = true;
    for (uint i = 0; i < size(); ++i)
    {
        if (!_lanes[i].flipBackwards()) success = false;
    }
    return success;
}



lane* section::operator[](size_t index)
{
    return &(_lanes[index]);
}

const lane* section::getLane(size_t index) const
{
    return &(_lanes[index]);
}

const lane* section::getOdrLane(size_t id) const
{
    for (uint i = 0; i < _writtenSize; ++i)
    {
        if (_lanes[i].odrID() == id)
            return &(_lanes[i]);
    }
    return nullptr;
}

lane* section::zero()
{
    return &_zero;
}

size_t section::size() const
{
    return _writtenSize;
}

size_t section::getMaxSize() const
{
    return _allocSize;
}


int section::getID() const
{
    return _id;
}

std::string section::getCSUID() const
{
    std::string id = std::to_string(_id);
    if (_writtenSize > 0)
    {
        if (_lanes[0].isOpenDrive())
            id += " (" + std::to_string(_odrID) + ")";
        else if (_lanes[0].isOneVersion())
            id += " (" + std::to_string(_ovID.nnodeID) + ":"
                    + std::to_string(_ovID.roadIDM) + "."
                    + std::to_string(_ovID.roadIDm) + ":"
                    + std::to_string(_ovID.lgIndex) + ")";
    }
    return id;
}

void section::setID(int id)
{
    _id = id;
}

bool section::isSameSection(const section *s) const
{
    if (s == this) return true;

    return false;
}

bool section::isConnected(const section &s) const
{
    for (uint i = 0; i < _writtenSize; ++i)
    {
        for (uint j = 0; j < s.size(); ++j)
            if (_lanes[i].isConnectedButNotThis( s.getLane(j) )) return true;
    }
    return false;
}


bool section::isCrosswalk() const
{
    if (size() == 0) return false;
    for (uint i = 0; i < size(); ++i)
        if (!_lanes[i].isCrosswalk()) return false;

    return true;
}

// We'll move to the left (port) most lane, and then walk rightwards (starboard-wards)
//  setting port/starboard pairs if the lanes have the same type.
// We cannot use Origin or Destination as they may be points shared by several lanes within the same section.
//   Therefore we use a point that's slightly ahead.
void section::setPortAndStarboard(bool assumeLeftHandDriving, bool assumeRightHandDriving)
{

    if (size() <= 1) return;

    // Get the port-most lane:
    // Start with lane Zero:
    scalar ahead = 1;
    if (_lanes[0].getLength() < ahead) ahead = 0.5 * _lanes[0].getLength();

    arr2 o;
    if (!_lanes[0].getPointAtDistance(o, ahead))
    {
        std::cerr << "[ Error ] lrn failed in determining port/starboard lanes " << _lanes[0].getCSUID() << std::endl;
    }
    arr2 to = _lanes[0].getTangentInPoint(o);

    int leftMost = 0; // if we don't find anything, it means it's Zero:
    // For each lane, check whether it is on the left, and if it is, take it:
    for (int j = 1; j < static_cast<int>(size()); ++j)
    {
        ahead = 1;
        if (_lanes[j].getLength() < ahead) ahead = 0.5 * _lanes[j].getLength();

        arr2 oj;
        _lanes[j].getPointAtDistance(oj, ahead);
        /* if (!_lanes[j].getPointAfterDistance(oj, _lanes[j].getOrigin(), ahead))
             * // this seemed mostly fine except for lane 29:1 in mira_pg_im4.xodr; still port/starboard are determined correctly even in that case.
            {
                std::cerr << "[ Error ] lrn failed in determining port/starboard lanes" << _lanes[j].getSUID() << std::endl;
                // return false;
            } */
        arr2 no = {oj[0] - o[0], oj[1] - o[1]}; // now;
        mvf::normalise(no); // and now check the angle:
        scalar alpha = mvf::subtendedAngle(to, no);

        if ((alpha > 0) && (alpha < ct::pi)) // that means left; the range of atan2 is [-pi, pi]
        {
            to = _lanes[j].getTangentInPoint(oj);
            leftMost = j;
            o = oj;
        }
    }
    // std::cout << "[ lrn ] leftMost lane: " << _lanes[leftMost].getSUID() << std::endl;

    std::unique_ptr<int[]> leftToRight(new int[size()]);
    leftToRight[0] = leftMost;
    for (uint i = 1; i < size(); ++i) leftToRight[i] = -1; // initialise the assigned lane as "-1"


    // Now starting from the leftMost, walk rightwards in the same way
    bool possible = true;
    for (uint i = 1; i < size(); ++i) // while (possible)
    {
        possible = false;
        // Find the closest lane rightwards:
        scalar dmin = 1e4;
        uint nextRight = leftMost;
        arr2 oNextRight;
        for (int j = 0; j < static_cast<int>(size()); ++j)
        {
            // Discard j if it was already stored. All this safety is needed because you'll be given crap in the maps.
            //   The last one was repeated lanes...
            bool alreadyStored = false;
            for (uint k = 0; k < i; ++k)
            {
                 if (j == leftToRight[k])
                 {
                     alreadyStored = true;
                     break;
                 }
            }
            if (alreadyStored) continue;

            ahead = 1;
            if (_lanes[j].getLength() < ahead) ahead = 0.5 * _lanes[j].getLength();

            arr2 oj;
            _lanes[j].getPointAtDistance(oj, ahead);
            arr2 no = {oj[0] - o[0], oj[1] - o[1]}; // now;
            scalar dj = mvf::magnitude(no);
            no = {no[0] / dj, no[1] / dj }; // normalise
            scalar alpha = mvf::subtendedAngle(to, no);

            if ((alpha < 0) && (dj < dmin)) // that means right; the range of atan2 is [-pi, pi]
            {
                dmin = dj;
                nextRight = j;
                oNextRight = oj;
                possible = true;
            }
        }

        if ((possible) && (_lanes[leftMost].getKind() == _lanes[nextRight].getKind()))
        {
            _lanes[nextRight].setPortLane(&(_lanes[leftMost]));
            _lanes[leftMost].setStarboardLane(&(_lanes[nextRight]));
            // std::cout << _lanes[nextRight].getSUID() << " has port lane: " << _lanes[leftMost].getSUID() << std::endl;
            // std::cout << _lanes[leftMost].getSUID() << " has starboard lane: " << _lanes[nextRight].getSUID() << std::endl;
        }

        if (possible) leftToRight[i] = nextRight;
        else break;

        to = _lanes[nextRight].getTangentInPoint(oNextRight);
        o = oNextRight;
        leftMost = nextRight;
        // std::cout << " next right: " << _lanes[nextRight].getSUID() << std::endl;
    }

    // DEDUCE The Direction of the Lanes:
    //  We firstly need some assumption:
    if ((!assumeLeftHandDriving) && (!assumeLeftHandDriving))
        return;
    else if ((assumeLeftHandDriving) && (assumeRightHandDriving))
    {
        std::cerr << "[ section ] cannot assume leftHandDriving and rightHandDriving; assuming nothing" << std::endl;
        return;
    }


    bool deduceDirection = false; // we may not deduce anything if every lane has the same direction.
    lane::sign so = _lanes[leftToRight[0]].getSign();
    for (uint i = 1; i < size(); ++i)
    {
        if (leftToRight[i] == -1) break;
        if (!_lanes[leftToRight[i]].isSameSign(so))
        {
            deduceDirection = true;
            break;
        }
    }

    if (!deduceDirection) return;

    if (assumeLeftHandDriving)
    {
        _lanes[leftToRight[0]].lockFlippable();
        // std::cout << "[ lane ] " << _lanes[leftToRight[0]].getSUID() << " goes from: "
                  // << _lanes[leftToRight[0]].getOrigin()[0] << ", " << _lanes[leftToRight[0]].getOrigin()[1] << " to "
                  // << _lanes[leftToRight[0]].getDestination()[0] << ", " << _lanes[leftToRight[0]].getDestination()[1] << " to "
                  // << std::endl;
        for (uint i = 1; i < size(); ++i)
        {
            if (leftToRight[i] == -1) break;
            if (!_lanes[leftToRight[i]].isSameSign(so))
                _lanes[leftToRight[i]].flipBackwards();

            _lanes[leftToRight[i]].lockFlippable();

            // std::cout << "[ lane ] " << _lanes[leftToRight[i]].getSUID() << " goes from: "
                  // << _lanes[leftToRight[i]].getOrigin()[0] << ", " << _lanes[leftToRight[i]].getOrigin()[1] << " to "
                  // << _lanes[leftToRight[i]].getDestination()[0] << ", " << _lanes[leftToRight[i]].getDestination()[1] << " to "
                  // << std::endl;
        }
    }

    else if (assumeRightHandDriving)
    {
        _lanes[leftToRight[0]].flipBackwards();
        for (uint i = 1; i < size(); ++i)
        {
            if (leftToRight[i] == -1) break;
            if (_lanes[leftToRight[i]].isSameSign(so))
                _lanes[leftToRight[i]].flipBackwards();

            _lanes[leftToRight[i]].lockFlippable();

        }

    }

}

void section::setPortAndStarboard(concepts::drivingSide ds)
{
    if (ds == concepts::drivingSide::leftHand)
        return setPortAndStarboard(true, false);
    else if (ds == concepts::drivingSide::rightHand)
        return setPortAndStarboard(false, true);
    else
        return setPortAndStarboard(false, false);
}


void section::lockOdrFlippable()
{
    for (uint i = 0; i < size(); ++i)
        _lanes[i].lockFlippable();

}

bool section::isTransitable()
{
    for (uint i = 0; i < size(); ++i)
        if (_lanes[i].isTransitable()) return true;
    return false;
}


bool section::isInOdrRange(scalar s) const
{
    return (_lanes[0].isInOdrRange(s));
}
